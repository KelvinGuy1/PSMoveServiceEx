// -- includes -----
#include "PS3EyeTracker.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "PSEyeVideoCapture.h"
#include "V4L2/V4L2VideoCapture.h"
#include <linux/videodev2.h>
#include "PSMoveProtocol.pb.h"
#include "DeviceManager.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerManager.h"
#include <fstream>

namespace {
    constexpr int CAP_MODE_BGR  = 0;
    constexpr int CAP_MODE_RGB  = 1;
    constexpr int CAP_MODE_GRAY = 2;
    constexpr int CAP_MODE_YUYV = 3;

    bool getUsbVendorProduct(const std::string &devicePath, int &outVendorId, int &outProductId)
    {
        std::string videoName = devicePath.substr(devicePath.find_last_of('/') + 1);
        std::string sysDevLink = "/sys/class/video4linux/" + videoName + "/device";

        char resolved[PATH_MAX];
        if (realpath(sysDevLink.c_str(), resolved) == nullptr)
            return false;

        std::string dir(resolved);

        for (int depth = 0; depth < 6; ++depth)
        {
            std::ifstream vendorFile(dir + "/idVendor");
            std::ifstream productFile(dir + "/idProduct");

            if (vendorFile.good() && productFile.good())
            {
                std::string vendorHex, productHex;
                std::getline(vendorFile, vendorHex);
                std::getline(productFile, productHex);

                if (!vendorHex.empty() && !productHex.empty())
                {
                    outVendorId = static_cast<int>(strtol(vendorHex.c_str(), nullptr, 16));
                    outProductId = static_cast<int>(strtol(productHex.c_str(), nullptr, 16));
                    return true;
                }
            }

            size_t lastSlash = dir.find_last_of('/');
            if (lastSlash == std::string::npos || lastSlash == 0)
                break;
            dir = dir.substr(0, lastSlash);
        }
        return false;
    }
}
#include "opencv2/opencv.hpp"

// -- constants -----
#define PS3EYE_STATE_BUFFER_MAX 16

static const char *OPTION_FOV_SETTING = "FOV Setting";
static const char *OPTION_FOV_RED_DOT = "Red Dot";
static const char *OPTION_FOV_BLUE_DOT = "Blue Dot";

// -- private definitions -----
class PSEyeCaptureData
{
public:
    PSEyeCaptureData()
    : frameHeight(-1)
    , frameWidth(-1)
    {
    }

    int frameHeight;
    int frameWidth;
    cv::Mat frame;
};

// -- public methods
// -- PS3EYE Controller Config
const int PS3EyeTrackerConfig::CONFIG_VERSION = 7;
const int PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION= 1;

PS3EyeTrackerConfig::PS3EyeTrackerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_timeout_ms(3000)
	, frame_width(640)
	, frame_height(480)
	, frame_rate(30)
    , exposure(32)
    , gain(32)
    , focalLengthX(554.2563) // pixels
    , focalLengthY(554.2563) // pixels
    , principalX(320.0) // pixels
    , principalY(240.0) // pixels
    , hfov(60.0) // degrees
    , vfov(45.0) // degrees
    , zNear(10.0) // cm
    , zFar(200.0) // cm
    , distortionK1(-0.10771770030260086)
    , distortionK2(0.1213262677192688)
    , distortionK3(0.04875476285815239)
    , distortionP1(0.00091733073350042105)
    , distortionP2(0.00010589254816295579)
    , fovSetting(BlueDot)
{
	memset(projection_blacklist, 0, sizeof(projection_blacklist));

    pose.clear();

	SharedColorPresets.table_name.clear();
	for (int i = 0; i < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++i)
	{
		SharedColorPresets.color_presets[i] = k_default_color_presets[i];
	}
};

const boost::property_tree::ptree
PS3EyeTrackerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);

    pt.put("version", PS3EyeTrackerConfig::CONFIG_VERSION);
	pt.put("legacy", DeviceManager().getInstance()->isLegacyService());

	pt.put("lens_calibration_version", PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION);
    pt.put("max_poll_failure_timeout_ms", max_poll_failure_timeout_ms);
	pt.put("frame_width", frame_width);
	pt.put("frame_height", frame_height);
	pt.put("frame_rate", frame_rate);
    pt.put("exposure", exposure);
	pt.put("gain", gain);
    pt.put("focalLengthX", focalLengthX);
    pt.put("focalLengthY", focalLengthY);
    pt.put("principalX", principalX);
    pt.put("principalY", principalY);
    pt.put("hfov", hfov);
    pt.put("vfov", vfov);
    pt.put("zNear", zNear);
    pt.put("zFar", zFar);
    pt.put("distortionK1", distortionK1);
    pt.put("distortionK2", distortionK2);
    pt.put("distortionK3", distortionK3);
    pt.put("distortionP1", distortionP1);
    pt.put("distortionP2", distortionP2);
    pt.put("fovSetting", static_cast<int>(fovSetting));

    pt.put("pose.orientation.w", pose.Orientation.w);
    pt.put("pose.orientation.x", pose.Orientation.x);
    pt.put("pose.orientation.y", pose.Orientation.y);
    pt.put("pose.orientation.z", pose.Orientation.z);
    pt.put("pose.position.x", pose.PositionCm.x);
    pt.put("pose.position.y", pose.PositionCm.y);
    pt.put("pose.position.z", pose.PositionCm.z);

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

	for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
	{
		std::string key_name = "blacklist_projections.";
		key_name.append(std::to_string(i));
		
		pt.put(key_name + std::string(".x"), projection_blacklist[i].x);
		pt.put(key_name + std::string(".y"), projection_blacklist[i].y);
		pt.put(key_name + std::string(".w"), projection_blacklist[i].w);
		pt.put(key_name + std::string(".h"), projection_blacklist[i].h);
	}
    return pt;
}

void
PS3EyeTrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    int version = pt.get<int>("version", 0);
	bool legacy = pt.get<bool>("legacy", false);

	if (version == PS3EyeTrackerConfig::CONFIG_VERSION && legacy == DeviceManager().getInstance()->isLegacyService())
    {
        is_valid = pt.get<bool>("is_valid", false);
		max_poll_failure_timeout_ms = pt.get<long>("max_poll_failure_timeout_ms", max_poll_failure_timeout_ms);
		frame_width = pt.get<double>("frame_width", frame_width);
		frame_height = pt.get<double>("frame_height", frame_height);
		frame_rate = pt.get<double>("frame_rate", frame_rate);
        exposure = pt.get<double>("exposure", exposure);
		gain = pt.get<double>("gain", gain);
        hfov = pt.get<double>("hfov", hfov);
        vfov = pt.get<double>("vfov", vfov);
        zNear = pt.get<double>("zNear", zNear);
        zFar = pt.get<double>("zFar", zFar);

		int lens_calibration_version = pt.get<int>("lens_calibration_version", 0);
		if (lens_calibration_version == PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION)
		{
			focalLengthX = pt.get<double>("focalLengthX", focalLengthX);
			focalLengthY = pt.get<double>("focalLengthY", focalLengthY);
			principalX = pt.get<double>("principalX", principalX);
			principalY = pt.get<double>("principalY", principalY);
			distortionK1 = pt.get<double>("distortionK1", distortionK1);
			distortionK2 = pt.get<double>("distortionK2", distortionK2);
			distortionK3 = pt.get<double>("distortionK3", distortionK3);
			distortionP1 = pt.get<double>("distortionP1", distortionP1);
			distortionP2 = pt.get<double>("distortionP2", distortionP2);
		}
		else
		{
			SERVER_LOG_WARNING("PS3EyeTrackerConfig") <<
				"Config version " << lens_calibration_version << " does not match expected version " <<
				PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION << ", Using defaults.";
		}

        fovSetting = 
            static_cast<PS3EyeTrackerConfig::eFOVSetting>(
                pt.get<int>("fovSetting", PS3EyeTrackerConfig::eFOVSetting::BlueDot));

        pose.Orientation.w = pt.get<float>("pose.orientation.w", 1.0);
        pose.Orientation.x = pt.get<float>("pose.orientation.x", 0.0);
        pose.Orientation.y = pt.get<float>("pose.orientation.y", 0.0);
        pose.Orientation.z = pt.get<float>("pose.orientation.z", 0.0);
        pose.PositionCm.x = pt.get<float>("pose.position.x", 0.0);
        pose.PositionCm.y = pt.get<float>("pose.position.y", 0.0);
        pose.PositionCm.z = pt.get<float>("pose.position.z", 0.0);

		// Read the default preset table
		readColorPropertyPresetTable(pt, &SharedColorPresets);

		// Read all of the controller preset tables
		const std::string controller_prefix("controller_");
		const std::string hmd_prefix("hmd_");
		for(auto iter = pt.begin(); iter != pt.end(); iter++)
		{
			const std::string &entry_name= iter->first;
			
			if (entry_name.compare(0, controller_prefix.length(), controller_prefix) == 0 ||
				entry_name.compare(0, hmd_prefix.length(), hmd_prefix) == 0)
			{
				CommonHSVColorRangeTable table;

				table.table_name= entry_name;
				for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
				{
					table.color_presets[preset_index] = k_default_color_presets[preset_index];
				}

				readColorPropertyPresetTable(pt, &table);

				DeviceColorPresets.push_back(table);
			}
		}

		for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
		{
			std::string key_name = "blacklist_projections.";
			key_name.append(std::to_string(i));

			projection_blacklist[i].clear();

			projection_blacklist[i].x = pt.get<float>(key_name + std::string(".x"), 0.0);
			projection_blacklist[i].y = pt.get<float>(key_name + std::string(".y"), 0.0);
			projection_blacklist[i].w = pt.get<float>(key_name + std::string(".w"), 0.0);
			projection_blacklist[i].h = pt.get<float>(key_name + std::string(".h"), 0.0);
		}
    }
    else
    {
        SERVER_LOG_WARNING("PS3EyeTrackerConfig") <<
            "Config version " << version << " does not match expected version " <<
            PS3EyeTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const CommonHSVColorRangeTable *
PS3EyeTrackerConfig::getColorRangeTable(const std::string &table_name) const
{
	const CommonHSVColorRangeTable *table= &SharedColorPresets;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}
	}

	return table;
}

inline CommonHSVColorRangeTable *
PS3EyeTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
{
	CommonHSVColorRangeTable *table= nullptr;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}

		if (table == nullptr)
		{
			CommonHSVColorRangeTable Table;

			Table.table_name= table_name;
			for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
			{
				Table.color_presets[preset_index] = k_default_color_presets[preset_index];
			}

			DeviceColorPresets.push_back(Table);
			table= &DeviceColorPresets[DeviceColorPresets.size() - 1];
		}
	}
	else
	{
		table= &SharedColorPresets;
	}

	return table;
}

// -- PS3EYE Tracker
PS3EyeTracker::PS3EyeTracker()
    : cfg()
    , USBDevicePath()
    , VideoCapture(nullptr)
    , CaptureData(nullptr)
    , m_captureBackend(Backend_PSEye)
    , m_v4l2Capture(nullptr)
    , DriverType(PS3EyeTracker::Libusb)
    , NextPollSequenceNumber(0)
    , TrackerStates()
{
}

PS3EyeTracker::~PS3EyeTracker()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PS3EyeTracker") << "Tracker deleted without calling close() first!";
    }
}

// -- IDeviceInterface
bool PS3EyeTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonControllerState::PS3EYE ||
		pEnum->get_device_type() == CommonControllerState::VirtualTracker)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == USBDevicePath);
    }

    return matches;
}

bool PS3EyeTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();

    bool bSuccess = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("PS3EyeTracker::open") << "PS3EyeTracker(" << cur_dev_path << ") already open. Ignoring request.";
        bSuccess = true;
    }
    else
    {
        const int camera_index = tracker_enumerator->get_camera_index();

		SERVER_LOG_INFO("PS3EyeTracker::open") << "DEBUG device_type=" << (int)tracker_enumerator->get_device_type() << " path=" << cur_dev_path;
		switch (tracker_enumerator->get_device_type())
		{
            case CommonDeviceState::eDeviceType::PS3EYE:
            {
                std::string devPathStr(cur_dev_path);
                bool isVideoNode = (devPathStr.find("/dev/video") == 0);

                if (isVideoNode)
                {
                    // Use V4L2 for any /dev/video* node (this includes PS3Eye cameras now)
                    SERVER_LOG_INFO("PS3EyeTracker::open") << "Opening V4L2VideoCapture(" << cur_dev_path << ", camera_index=" << camera_index << ")";

                    m_captureBackend = Backend_V4L2;
                    m_v4l2Capture = new V4L2VideoCapture();

                    if (m_v4l2Capture->open(cur_dev_path, 640, 480, 30))
                    {
                        CaptureData = new PSEyeCaptureData;
                        CaptureData->frame = cv::Mat::zeros(480, 640, CV_8UC3);
                        USBDevicePath = enumerator->get_path();
                        bSuccess = true;
                    }
                    else
                    {
                        SERVER_LOG_ERROR("PS3EyeTracker::open") << "Failed to open V4L2VideoCapture(" << cur_dev_path << ", camera_index=" << camera_index << ")";
                        close();
                    }
                }
                else
                {
                    // USB path (like USB\VID_1415...) – skip it; the generic enumerator will handle the /dev/video node.
                    SERVER_LOG_WARNING("PS3EyeTracker::open") << "Skipping USB path for PS3Eye: " << cur_dev_path;
                    bSuccess = false;
                }
                break;
            }
			case CommonDeviceState::eDeviceType::VirtualTracker:
			{
				const int camera_virt_index = tracker_enumerator->get_camera_virt_index();

				SERVER_LOG_INFO("PS3EyeTracker::open") << "Opening PSEyeVideoCapture(" << cur_dev_path << ", camera_virt_index=" << camera_virt_index << ", camera_index=" << camera_index << ")";

				VideoCapture = new PSEyeVideoCapture(camera_virt_index, camera_index, PSEyeVideoCapture::eVideoCaptureType::CaptureType_VIRTUAL);

				if (VideoCapture->isOpened())
				{
					CaptureData = new PSEyeCaptureData;
					USBDevicePath = enumerator->get_path();
					bSuccess = true;
				}
				else
				{
					SERVER_LOG_ERROR("PS3EyeTracker::open") << "Failed to open PS3EyeTracker(" << cur_dev_path << ", camera_virt_index=" << camera_virt_index << ", camera_index=" << camera_index << ")";

					close();
				}
				break;
			}
		}
    }

    if (bSuccess)
    {
        std::string identifier;
        if (m_captureBackend == Backend_V4L2)
        {
            // Use persistent USB port identifier instead of /dev/videoN
            identifier = m_v4l2Capture->getPersistentIdentifier();
            // Sanitize (replace slashes and backslashes)
            for (char &c : identifier)
            {
                if (c == '/' || c == '\\')
                    c = '_';
            }
        }
        else
        {
                identifier = VideoCapture->getUniqueIndentifier();
        }
        std::string config_name = "PS3EyeTrackerConfig_";
        config_name.append(identifier);

        cfg = PS3EyeTrackerConfig(config_name);
        cfg.load();
        cfg.save();

        if (m_captureBackend == Backend_PSEye)
        {
            const double currentFrameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
            const double currentFrameRate = VideoCapture->get(cv::CAP_PROP_FPS);
            const double currentExposure = VideoCapture->get(cv::CAP_PROP_EXPOSURE);
            const double currentGain = VideoCapture->get(cv::CAP_PROP_GAIN);

            if (currentFrameWidth != cfg.frame_width)
                VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, cfg.frame_width);

            if (currentExposure != cfg.exposure)
                VideoCapture->set(cv::CAP_PROP_EXPOSURE, cfg.exposure);

            if (currentGain != cfg.gain)
                VideoCapture->set(cv::CAP_PROP_GAIN, cfg.gain);

            if (currentFrameRate != cfg.frame_rate)
                VideoCapture->set(cv::CAP_PROP_FPS, cfg.frame_rate);
            VideoCapture->set(CV_CAP_PROP_MAXFAILPOLL, cfg.max_poll_failure_timeout_ms);
        }
        else if (m_captureBackend == Backend_V4L2)
        {
            // Reapply framerate from config
            if (m_v4l2Capture)
            {
                m_v4l2Capture->setFramerate(static_cast<int>(cfg.frame_rate));

                // Reapply resolution if it differs from current
                int currentWidth = m_v4l2Capture->getWidth();
                int currentHeight = m_v4l2Capture->getHeight();
                if (currentWidth != static_cast<int>(cfg.frame_width) ||
                    currentHeight != static_cast<int>(cfg.frame_height))
                {
                    m_v4l2Capture->setResolution(static_cast<int>(cfg.frame_width),
                                                 static_cast<int>(cfg.frame_height));
                }
            }
            // Resolution is already negotiated at open, no need to reapply
        }
    }


    return bSuccess;
}

bool PS3EyeTracker::getIsOpen() const
{
    return VideoCapture != nullptr || m_v4l2Capture != nullptr;
}

bool PS3EyeTracker::getIsReadyToPoll() const
{
    return getIsOpen();
}

IDeviceInterface::ePollResult PS3EyeTracker::poll()
{
    IDeviceInterface::ePollResult result = IDeviceInterface::_PollResultFailure;

    if (getIsOpen())
    {
                if (m_captureBackend == Backend_V4L2)
                {
                        // Generic UVC webcam via direct V4L2/MJPEG capture: no sync gate,
                        // no property-flag dance, no GStreamer instability. Just a blocking read.
                        if (m_v4l2Capture->read(CaptureData->frame) && !CaptureData->frame.empty())
                        {
                            CaptureData->frameWidth = m_v4l2Capture->getWidth();
                            CaptureData->frameHeight = m_v4l2Capture->getHeight();
                            DeviceManager::getInstance()->m_tracker_manager->setTrackerReady(m_v4l2Capture->getIndex());
                            result = IControllerInterface::_PollResultSuccessNewData;
                        }
                        else
                        {
                            // If read failed, keep a black frame to avoid downstream crashes
                            CaptureData->frame = cv::Mat::zeros(480, 640, CV_8UC3);
                            CaptureData->frameWidth = 640;
                            CaptureData->frameHeight = 480;
                            result = IControllerInterface::_PollResultSuccessIgnore;
                        }
                }
                else
                {
		// Prepare frames whenever we can.
                bool isCustomCaptureDomain = VideoCapture->isUsingCustomCapture();

                if (isCustomCaptureDomain)
                {
                        // Custom PS3Eye-family capture wrappers use a two-step grab()/retrieve()
                        // protocol with a custom CV_CAP_PROP_FRAMEAVAILABLE sync flag.
                        if (VideoCapture->grab())
                        {
                                if ((bool)VideoCapture->get(CV_CAP_PROP_FRAMEAVAILABLE))
                                {
                                        DeviceManager::getInstance()->m_tracker_manager->setTrackerReady(VideoCapture->getIndex());
                                }

                                // Only poll frames when every tracker is ready to sync freams.
                                if (!DeviceManager::getInstance()->m_tracker_manager->isTrackerPollAllowed())
                                {
                                        // Keep iterating. Still has old data.
                                        result = IControllerInterface::_PollResultSuccessIgnore;
                                }
                                else
                                {
                                        if (!VideoCapture->retrieve(CaptureData->frame, cv::CAP_OPENNI_BGR_IMAGE))
                                        {
                                                // Device still in valid state
                                                result = IControllerInterface::_PollResultSuccessIgnore;
                                        }
                                        else
                                        {
                                                CaptureData->frameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
                                                CaptureData->frameHeight = VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT);

                                                // New data available. Keep iterating.
                                                result = IControllerInterface::_PollResultSuccessNewData;
                                                // We received the frame and every tracker polled. We need a new frame!
                                                VideoCapture->set(CV_CAP_PROP_FRAMEAVAILABLE, false);
                                        }
                                }
                        }
                        else
                        {
                                result = IControllerInterface::_PollResultSuccessIgnore;
                        }
                }
                else
                {
                        // Generic UVC/GStreamer webcam: skip the PS3Eye-specific grab()/flag/retrieve()
                        // protocol AND the multi-tracker sync gate (irrelevant for a single generic camera)
                        // entirely, and use OpenCV's standard combined blocking read().
                        DeviceManager::getInstance()->m_tracker_manager->setTrackerReady(VideoCapture->getIndex());

                        if (VideoCapture->read(CaptureData->frame) && !CaptureData->frame.empty())
                        {
                            CaptureData->frameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
                            CaptureData->frameHeight = VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT);

                                result = IControllerInterface::_PollResultSuccessNewData;
                        }
                        else
                        {
                                result = IControllerInterface::_PollResultSuccessIgnore;
                        }
                }

                }

        {
            PS3EyeTrackerState newState;

            // TODO: Process the frame and extract the blobs

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = NextPollSequenceNumber;
			newState.DeviceType = getDeviceType();
            ++NextPollSequenceNumber;

            // Make room for new entry if at the max queue size
            //###bwalker $TODO Make this a fixed size circular buffer
            if (TrackerStates.size() >= PS3EYE_STATE_BUFFER_MAX)
            {
                TrackerStates.erase(TrackerStates.begin(), TrackerStates.begin() + TrackerStates.size() - PS3EYE_STATE_BUFFER_MAX);
            }

            TrackerStates.push_back(newState);
        }
    }

    return result;
}

void PS3EyeTracker::close()
{
    if (CaptureData != nullptr)
    {
        delete CaptureData;
        CaptureData = nullptr;
    }

    if (VideoCapture != nullptr)
    {
        delete VideoCapture;
        VideoCapture = nullptr;
    }

    if (m_v4l2Capture != nullptr)
    {
        delete m_v4l2Capture;
        m_v4l2Capture = nullptr;
    }

    m_captureBackend = Backend_PSEye;
}

long PS3EyeTracker::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_timeout_ms;
}

CommonDeviceState::eDeviceType PS3EyeTracker::getDeviceType() const
{
	// Virtual trackers have a common device path "VirtualTracker_#"
	if (USBDevicePath[0] == 'V')
		return CommonDeviceState::VirtualTracker;

    return CommonDeviceState::PS3EYE;
}

const CommonDeviceState *PS3EyeTracker::getState(int lookBack) const
{
    const int queueSize = static_cast<int>(TrackerStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &TrackerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

ITrackerInterface::eDriverType PS3EyeTracker::getDriverType() const
{
    //###bwalker $TODO Get the driver type from VideoCapture
    return DriverType;
}

std::string PS3EyeTracker::getUSBDevicePath() const
{
    return USBDevicePath;
}

bool PS3EyeTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    int width;
    int height;

    if (m_captureBackend == Backend_V4L2)
    {
        width = m_v4l2Capture->getWidth();
        height = m_v4l2Capture->getHeight();
    }
    else
    {
        width = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH));
        height = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));
    }

    if (out_width != nullptr)
    {
        if (out_stride != nullptr)
        {
            int bytes_per_pixel;

            if (m_captureBackend == Backend_V4L2)
            {
                // cv::imdecode() always produces a 3-channel BGR Mat for MJPEG.
                bytes_per_pixel = 3;
            }
            else
            {
                int format = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FORMAT));

                if (format != -1)
                {
                    switch (format)
                    {
                    case CAP_MODE_BGR:
                    case CAP_MODE_RGB:
                        bytes_per_pixel = 3;
                        break;
                    case CAP_MODE_YUYV:
                        bytes_per_pixel = 2;
                        break;
                    case CAP_MODE_GRAY:
                        bytes_per_pixel = 1;
                        break;
                    default:
                        assert(false && "Unknown video format?");
                        bytes_per_pixel = 3;
                        break;
                    }
                }
                else
                {
                    SERVER_LOG_ERROR("PS3EyeTracker::getVideoFrameDimensions") << "Unknown video format for camera" << USBDevicePath << ")";
                    bytes_per_pixel = 3;
                }
            }

            *out_stride = bytes_per_pixel * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        *out_height = height;
    }

    return bSuccess;
}

const unsigned char *
PS3EyeTracker::getVideoFrameBuffer(int &frameHeight, int &frameWidth) const
{
    if (CaptureData == nullptr ||
        CaptureData->frame.empty())
    {
        frameHeight = 480;
        frameWidth = 640;
        return nullptr;
    }

    frameHeight = CaptureData->frame.rows;
    frameWidth = CaptureData->frame.cols;

    return CaptureData->frame.data;
}

void PS3EyeTracker::loadSettings()
{
    cfg.load();

    if (m_captureBackend == Backend_PSEye)
    {
        const double currentFrameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
        const double currentFrameRate = VideoCapture->get(cv::CAP_PROP_FPS);
        const double currentExposure = VideoCapture->get(cv::CAP_PROP_EXPOSURE);
        const double currentGain = VideoCapture->get(cv::CAP_PROP_GAIN);

        if (currentFrameWidth != cfg.frame_width)
            VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, cfg.frame_width);

        if (currentExposure != cfg.exposure)
            VideoCapture->set(cv::CAP_PROP_EXPOSURE, cfg.exposure);

        if (currentGain != cfg.gain)
            VideoCapture->set(cv::CAP_PROP_GAIN, cfg.gain);

        if (currentFrameRate != cfg.frame_rate)
            VideoCapture->set(cv::CAP_PROP_FPS, cfg.frame_rate);
    }
    else if (m_captureBackend == Backend_V4L2 && m_v4l2Capture)
    {
        // Reapply framerate from config
        m_v4l2Capture->setFramerate(static_cast<int>(cfg.frame_rate));

        // Reapply resolution if it differs from current
        int currentWidth = m_v4l2Capture->getWidth();
        int currentHeight = m_v4l2Capture->getHeight();
        if (currentWidth != static_cast<int>(cfg.frame_width) ||
            currentHeight != static_cast<int>(cfg.frame_height))
        {
            m_v4l2Capture->setResolution(static_cast<int>(cfg.frame_width),
                                         static_cast<int>(cfg.frame_height));
        }

        // Exposure and gain are already applied when the sliders move,
        // but we could reapply them here if needed.
    }
}

void PS3EyeTracker::saveSettings()
{
    cfg.save();
}

void PS3EyeTracker::setFrameWidth(double value, bool bUpdateConfig)
{
    if (m_captureBackend == Backend_V4L2)
    {
        // We need both width and height – use current height from config
        m_v4l2Capture->setResolution(static_cast<int>(value), static_cast<int>(cfg.frame_height));
    }
    else if (m_captureBackend == Backend_PSEye)
    {
        VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, value);
    }

    if (bUpdateConfig)
        cfg.frame_width = value;
}

double PS3EyeTracker::getFrameWidth() const
{
    if (m_captureBackend == Backend_V4L2)
    {
        return m_v4l2Capture->getWidth();
    }

    return VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
}



void PS3EyeTracker::setFrameHeight(double value, bool bUpdateConfig)
{
    if (m_captureBackend == Backend_V4L2)
    {
        m_v4l2Capture->setResolution(static_cast<int>(cfg.frame_width), static_cast<int>(value));
    }
    else if (m_captureBackend == Backend_PSEye)
    {
        VideoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, value);
    }

    if (bUpdateConfig)
        cfg.frame_height = value;
}

double PS3EyeTracker::getFrameHeight() const
{
    if (m_captureBackend == Backend_V4L2)
    {
        return m_v4l2Capture->getHeight();
    }

    return VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT);
}

void PS3EyeTracker::setFrameRate(double value, bool bUpdateConfig)
{
    if (m_captureBackend == Backend_V4L2)
    {
        if (m_v4l2Capture->setFramerate(static_cast<int>(value)))
        {
            SERVER_LOG_INFO("PS3EyeTracker::setFrameRate") << "Framerate set to " << value;
        }
        else
        {
            SERVER_LOG_WARNING("PS3EyeTracker::setFrameRate") << "Failed to set framerate to " << value << " – unsupported?";
        }
    }
    else if (m_captureBackend == Backend_PSEye && VideoCapture->isUsingCustomCapture())
    {
        VideoCapture->set(cv::CAP_PROP_FPS, value);
    }

    if (bUpdateConfig)
    {
        cfg.frame_rate = value;
        cfg.save(); // Ensure it's written to disk immediately
    }
}

double PS3EyeTracker::getFrameRate() const
{
        if (m_captureBackend == Backend_V4L2)
        {
                return m_v4l2Capture->getFps();
        }

        return VideoCapture->get(cv::CAP_PROP_FPS);
}

void PS3EyeTracker::setExposure(double value, bool bUpdateConfig)
{
    if (m_captureBackend == Backend_V4L2)
    {
        // Disable auto-exposure (if supported)
        if (m_v4l2Capture->isControlSupported(V4L2_CID_EXPOSURE_AUTO))
        {
            m_v4l2Capture->setControl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        }

        // Try absolute exposure (webcams often use this)
        if (m_v4l2Capture->isControlSupported(V4L2_CID_EXPOSURE_ABSOLUTE))
        {
            m_v4l2Capture->setControl(V4L2_CID_EXPOSURE_ABSOLUTE, static_cast<int32_t>(value));
        }
        // Fallback to plain exposure (PS3Eye)
        else if (m_v4l2Capture->isControlSupported(V4L2_CID_EXPOSURE))
        {
            m_v4l2Capture->setControl(V4L2_CID_EXPOSURE, static_cast<int32_t>(value));
        }
        else
        {
            SERVER_LOG_WARNING("PS3EyeTracker::setExposure") << "No exposure control supported for this V4L2 device";
        }
    }
    else if (m_captureBackend == Backend_PSEye && VideoCapture->isUsingCustomCapture())
    {
        VideoCapture->set(cv::CAP_PROP_EXPOSURE, value);
    }

    if (bUpdateConfig)
    {
        cfg.exposure = value;
    }
}


double PS3EyeTracker::getExposure() const
{
    if (m_captureBackend == Backend_V4L2)
    {
        int32_t value = 0;
        if (m_v4l2Capture->isControlSupported(V4L2_CID_EXPOSURE_ABSOLUTE) &&
            m_v4l2Capture->getControl(V4L2_CID_EXPOSURE_ABSOLUTE, value))
        {
            return static_cast<double>(value);
        }
        else if (m_v4l2Capture->isControlSupported(V4L2_CID_EXPOSURE) &&
            m_v4l2Capture->getControl(V4L2_CID_EXPOSURE, value))
        {
            return static_cast<double>(value);
        }
        return cfg.exposure;
    }
    return VideoCapture->get(cv::CAP_PROP_EXPOSURE);
}

void PS3EyeTracker::setGain(double value, bool bUpdateConfig)
{
    if (m_captureBackend == Backend_V4L2)
    {
        // Try gain first
        if (m_v4l2Capture->isControlSupported(V4L2_CID_GAIN))
        {
            // Disable auto-gain if supported
            if (m_v4l2Capture->isControlSupported(V4L2_CID_AUTOGAIN))
            {
                m_v4l2Capture->setControl(V4L2_CID_AUTOGAIN, 0);
            }
            m_v4l2Capture->setControl(V4L2_CID_GAIN, static_cast<int32_t>(value));
            SERVER_LOG_INFO("PS3EyeTracker::setGain") << "Setting gain to " << value;
        }
        // Fallback to brightness if gain is not supported
        else if (m_v4l2Capture->isControlSupported(V4L2_CID_BRIGHTNESS))
        {
            m_v4l2Capture->setControl(V4L2_CID_BRIGHTNESS, static_cast<int32_t>(value));
            SERVER_LOG_INFO("PS3EyeTracker::setGain") << "Gain not supported, setting brightness to " << value;
        }
        else
        {
            SERVER_LOG_WARNING("PS3EyeTracker::setGain") << "No gain or brightness control supported";
        }
    }
    else if (m_captureBackend == Backend_PSEye && VideoCapture->isUsingCustomCapture())
    {
        VideoCapture->set(cv::CAP_PROP_GAIN, value);
    }

    if (bUpdateConfig)
    {
        cfg.gain = value;
        cfg.save();
    }
}

double PS3EyeTracker::getGain() const
{
    if (m_captureBackend == Backend_V4L2)
    {
        int32_t value = 0;

        // Try gain first
        if (m_v4l2Capture->isControlSupported(V4L2_CID_GAIN) &&
            m_v4l2Capture->getControl(V4L2_CID_GAIN, value))
        {
            return static_cast<double>(value);
        }
        // Fallback to brightness
        else if (m_v4l2Capture->isControlSupported(V4L2_CID_BRIGHTNESS) &&
            m_v4l2Capture->getControl(V4L2_CID_BRIGHTNESS, value))
        {
            return static_cast<double>(value);
        }
        else
        {
            return cfg.gain; // return last known value
        }
    }
    return VideoCapture->get(cv::CAP_PROP_GAIN);
}

void PS3EyeTracker::getCameraIntrinsics(
    float &outFocalLengthX, float &outFocalLengthY,
    float &outPrincipalX, float &outPrincipalY,
    float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
    float &outDistortionP1, float &outDistortionP2) const
{
    SERVER_LOG_INFO("PS3EyeTracker::getCameraIntrinsics")
    << "cfg.focalLengthX=" << cfg.focalLengthX
    << " cfg.focalLengthY=" << cfg.focalLengthY
    << " cfg.principalX=" << cfg.principalX
    << " cfg.principalY=" << cfg.principalY
    << " cfg.distortionK1=" << cfg.distortionK1;

    if (CaptureData == nullptr || CaptureData->frame.empty())
    {
        outFocalLengthX = 554.2563f;
        outFocalLengthY = 554.2563f;
        outPrincipalX = 320.0f;
        outPrincipalY = 240.0f;
        outDistortionK1 = -0.1077177f;
        outDistortionK2 = 0.12132627f;
        outDistortionK3 = 0.04875476f;
        outDistortionP1 = 0.0009173307f;
        outDistortionP2 = 0.0001058925f;
        return;
    }
	// ###Externet $TODO Scale precomputed intrinsics if its other than 480p.
	// 480p should be the default resolution for intrinsics.
	const float NW = static_cast<float>(getFrameWidth() / 640);
	const float NH = static_cast<float>(getFrameHeight() / 480);

    outFocalLengthX = static_cast<float>(cfg.focalLengthX) * NW;
    outFocalLengthY = static_cast<float>(cfg.focalLengthY) * NH;
    outPrincipalX = static_cast<float>(cfg.principalX) * NW;
    outPrincipalY = static_cast<float>(cfg.principalY) * NH;
    outDistortionK1 = static_cast<float>(cfg.distortionK1);
    outDistortionK2 = static_cast<float>(cfg.distortionK2);
    outDistortionK3 = static_cast<float>(cfg.distortionK3);
    outDistortionP1 = static_cast<float>(cfg.distortionP1);
    outDistortionP2 = static_cast<float>(cfg.distortionP2);
}

void PS3EyeTracker::setCameraIntrinsics(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY,
    float distortionK1, float distortionK2, float distortionK3,
    float distortionP1, float distortionP2)
{
	// ###Externet $TODO Scale precomputed intrinsics if its other than 480p.
	// 480p should be the default resolution for intrinsics.
	const float NW = static_cast<float>(getFrameWidth() / 640);
	const float NH = static_cast<float>(getFrameHeight() / 480);

    cfg.focalLengthX = focalLengthX / NW;
    cfg.focalLengthY = focalLengthY / NH;
    cfg.principalX = principalX / NW;
    cfg.principalY = principalY / NH;
    cfg.distortionK1 = distortionK1;
    cfg.distortionK2 = distortionK2;
    cfg.distortionK3 = distortionK3;
    cfg.distortionP1 = distortionP1;
    cfg.distortionP2 = distortionP2;
}

CommonDevicePose PS3EyeTracker::getTrackerPose() const
{
    //SERVER_LOG_INFO("PS3EyeTracker::getTrackerPose")
    //<< "Returning pose: pos=("
    //<< cfg.pose.PositionCm.x << "," << cfg.pose.PositionCm.y << "," << cfg.pose.PositionCm.z
    //<< ") orient=(" << cfg.pose.Orientation.w << "," << cfg.pose.Orientation.x << ","
    //<< cfg.pose.Orientation.y << "," << cfg.pose.Orientation.z << ")";
    return cfg.pose;
}

void PS3EyeTracker::setTrackerPose(const struct CommonDevicePose *pose)
{
    SERVER_LOG_INFO("PS3EyeTracker::setTrackerPose")
    //<< "Setting pose: pos=("
    //<< pose->PositionCm.x << "," << pose->PositionCm.y << "," << pose->PositionCm.z
    //<< ") orient=(" << pose->Orientation.w << "," << pose->Orientation.x << ","
    //<< pose->Orientation.y << "," << pose->Orientation.z << ")";
    cfg.pose = *pose;
    cfg.save();
}

void PS3EyeTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(cfg.hfov);
    outVFOV = static_cast<float>(cfg.vfov);
}

void PS3EyeTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(cfg.zNear);
    outZFar = static_cast<float>(cfg.zFar);
}

void PS3EyeTracker::gatherTrackerOptions(
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
    PSMoveProtocol::OptionSet *optionSet = settings->add_option_sets();
    
    optionSet->set_option_name(OPTION_FOV_SETTING);
    optionSet->add_option_strings(OPTION_FOV_RED_DOT);
    optionSet->add_option_strings(OPTION_FOV_BLUE_DOT);
    optionSet->set_option_index(static_cast<int>(cfg.fovSetting));
}

bool PS3EyeTracker::setOptionIndex(
    const std::string &option_name,
    int option_index)
{
    bool bValidOption = false;

    if (option_name == OPTION_FOV_SETTING && 
        option_index >= 0 && 
        option_index < PS3EyeTrackerConfig::eFOVSetting::MAX_FOV_SETTINGS)
    {
        cfg.fovSetting = static_cast<PS3EyeTrackerConfig::eFOVSetting>(option_index);
        //###HipsterSloth $TODO Update the focal lengths?

        bValidOption = true;
    }

    return bValidOption;
}

bool PS3EyeTracker::getOptionIndex(
    const std::string &option_name, 
    int &out_option_index) const
{
    bool bValidOption = false;

    if (option_name == OPTION_FOV_SETTING)
    {
        out_option_index = static_cast<int>(cfg.fovSetting);
        bValidOption = true;
    }

    return bValidOption;
}

void PS3EyeTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    for (int list_index = 0; list_index < MAX_TRACKING_COLOR_TYPES; ++list_index)
    {
        const CommonHSVColorRange &hsvRange = table->color_presets[list_index];
        const eCommonTrackingColorID colorType = static_cast<eCommonTrackingColorID>(list_index);

        PSMoveProtocol::TrackingColorPreset *colorPreset= settings->add_color_presets();
        colorPreset->set_color_type(static_cast<PSMoveProtocol::TrackingColorType>(colorType));
        colorPreset->set_hue_center(hsvRange.hue_range.center);
        colorPreset->set_hue_range(hsvRange.hue_range.range);
        colorPreset->set_saturation_center(hsvRange.saturation_range.center);
        colorPreset->set_saturation_range(hsvRange.saturation_range.range);
        colorPreset->set_value_center(hsvRange.value_range.center);
        colorPreset->set_value_range(hsvRange.value_range.range);
    }
}

void PS3EyeTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    const CommonHSVColorRange *preset)
{
//    cfg.ColorPresets[color] = *preset; // from generic_camera conflict
	CommonHSVColorRangeTable *table= cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    cfg.save();
}

void PS3EyeTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    CommonHSVColorRange *out_preset) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

void PS3EyeTracker::setBlacklistProjection(const int index, const float x, const float y, const float w, const float h)
{
	if (index < 0 || index > eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS - 1)
	{
		return;
	}

	cfg.projection_blacklist[index].clear();
	cfg.projection_blacklist[index].x = x;
	cfg.projection_blacklist[index].y = y;
	cfg.projection_blacklist[index].w = w;
	cfg.projection_blacklist[index].h = h;
}

bool PS3EyeTracker::getBlacklistProjection(const int index, float &x, float &y, float &w, float &h) const
{
	if (index < 0 || index > eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS - 1)
	{
		return false;
	}

	x = cfg.projection_blacklist[index].x;
	y = cfg.projection_blacklist[index].y;
	w = cfg.projection_blacklist[index].w;
	h = cfg.projection_blacklist[index].h;
	return true;
}
