#include "V4L2VideoCapture.h"

#include <opencv2/imgcodecs.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <cstring>
#include <cerrno>
#include <dirent.h>
#include <algorithm>
#include <fstream>

#ifdef SERVER_LOG_H
#include "ServerLog.h"
#define V4L2_LOG_INFO(tag) SERVER_LOG_INFO(tag)
#define V4L2_LOG_ERROR(tag) SERVER_LOG_ERROR(tag)
#define V4L2_LOG_WARNING(tag) SERVER_LOG_WARNING(tag)   // <-- add this line
#else
#include <iostream>
#define V4L2_LOG_INFO(tag) std::cout << "[" << tag << "] "
#define V4L2_LOG_ERROR(tag) std::cerr << "[ERROR] [" << tag << "] "
#define V4L2_LOG_WARNING(tag) std::cerr << "[WARNING] [" << tag << "] "
#endif

namespace
{
    const int k_bufferCount = 4;

    int xioctl(int fd, unsigned long request, void *arg)
    {
        int r;
        do
        {
            r = ioctl(fd, request, arg);
        } while (r == -1 && errno == EINTR);
        return r;
    }

    static bool getUsbVendorProduct(const std::string &devicePath, int &outVendorId, int &outProductId)
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

V4L2VideoCapture::V4L2VideoCapture()
: m_fd(-1)
, m_width(0)
, m_height(0)
, m_fps(0)
, m_streaming(false)
{
}

V4L2VideoCapture::~V4L2VideoCapture()
{
    close();
}

std::string V4L2VideoCapture::getPersistentIdentifier() const
{
    std::string videoName = m_devicePath.substr(m_devicePath.find_last_of('/') + 1);
    std::string sysDevLink = "/sys/class/video4linux/" + videoName + "/device";

    char resolved[PATH_MAX];
    if (realpath(sysDevLink.c_str(), resolved) == nullptr)
        return m_devicePath; // fallback

        std::string dir(resolved);

    // Walk up until we find a directory containing "usb" in its path
    while (!dir.empty() && dir != "/")
    {
        if (dir.find("/usb") != std::string::npos)
        {
            // Extract the USB port path (e.g., "1-1.3")
            std::string usbPath = dir.substr(dir.find_last_of('/') + 1);
            // Remove any interface suffix (e.g., ":1.0")
            size_t colonPos = usbPath.find(':');
            if (colonPos != std::string::npos)
                usbPath = usbPath.substr(0, colonPos);
            return usbPath;
        }
        size_t lastSlash = dir.find_last_of('/');
        if (lastSlash == std::string::npos) break;
        dir = dir.substr(0, lastSlash);
    }

    // Fallback: use VID/PID if we can't get a USB port path
    int vendorId, productId;
    if (getUsbVendorProduct(m_devicePath, vendorId, productId))
    {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%04x_%04x", vendorId, productId);
        return std::string(buffer);
    }

    return m_devicePath; // final fallback
}

std::vector<std::string> V4L2VideoCapture::enumerateDevices()
{
    std::vector<std::string> result;

    DIR *dir = opendir("/dev");
    if (dir == nullptr)
    {
        return result;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name(entry->d_name);
        if (name.rfind("video", 0) != 0)
        {
            continue;
        }

        std::string path = "/dev/" + name;

        int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0)
        {
            continue;
        }

        struct v4l2_capability cap;
        std::memset(&cap, 0, sizeof(cap));

        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
        {
            bool canCapture = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0;
            bool canStream = (cap.capabilities & V4L2_CAP_STREAMING) != 0;

            if (canCapture && canStream)
            {
                result.push_back(path);
            }
        }

        ::close(fd);
    }

    closedir(dir);
    std::sort(result.begin(), result.end());

    return result;
}

bool V4L2VideoCapture::open(int index, int width, int height, int fps)
{
    std::vector<std::string> devices = enumerateDevices();

    if (index < 0 ||
        static_cast<size_t>(index) >= devices.size())
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::open")
        << "No capture device at index " << index
        << " (" << devices.size()
        << " device(s) found)" << std::endl;

        return false;
    }

    m_index = index;

    return open(devices[index], width, height, fps);
}

bool V4L2VideoCapture::open(const std::string &devicePath, int width, int height, int fps)
{
    close();

    if (!openDevice(devicePath, width, height, fps))
    {
        close();
        return false;
    }

    return true;
}

bool V4L2VideoCapture::openDevice(const std::string &devicePath, int width, int height, int fps)
{
    m_fd = ::open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (m_fd < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::openDevice") << "Failed to open " << devicePath
            << ": " << strerror(errno) << std::endl;
        return false;
    }

    m_devicePath = devicePath;

    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));

    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::openDevice") << devicePath << " is not a V4L2 device" << std::endl;
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING))
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::openDevice") << devicePath
            << " does not support streaming video capture" << std::endl;
        return false;
    }

    if (!negotiateFormat(width, height))
    {
        return false;
    }
    else
    {
        // Detect if this is a PS3Eye camera
        int vendorId = -1, productId = -1;
        if (getUsbVendorProduct(devicePath, vendorId, productId))
        {
            m_isPS3Eye = (vendorId == 0x1415 && productId == 0x2000);
        }
        else
        {
            m_isPS3Eye = false;
        }
    }

    struct v4l2_streamparm streamParm;
    std::memset(&streamParm, 0, sizeof(streamParm));
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamParm.parm.capture.timeperframe.numerator = 1;
    streamParm.parm.capture.timeperframe.denominator = fps;
    xioctl(m_fd, VIDIOC_S_PARM, &streamParm);
    m_fps = fps;

    if (!setupBuffers())
    {
        return false;
    }

    if (!startStreaming())
    {
        return false;
    }

    if (!m_isPS3Eye)
    {
        if (isControlSupported(V4L2_CID_FOCUS_AUTO))
        {
            //We run this multiple times to esure that it actually attempts it.
            setControl(V4L2_CID_FOCUS_AUTO, 0);
            usleep(100000);
            setControl(V4L2_CID_FOCUS_AUTO, 1);
            usleep(100000);
            setControl(V4L2_CID_FOCUS_AUTO, 0);
            usleep(100000);
            setControl(V4L2_CID_FOCUS_AUTO, 1);
        }
        else if (isControlSupported(V4L2_CID_FOCUS_ABSOLUTE))
        {
            int32_t min, max, def;
            if (getControlRange(V4L2_CID_FOCUS_ABSOLUTE, min, max, def))
            {
                setControl(V4L2_CID_FOCUS_ABSOLUTE, (min + max) / 2);
            }
        }
    }

    V4L2_LOG_INFO("V4L2VideoCapture::openDevice") << "Opened " << devicePath
    << " at " << m_width << "x" << m_height
    << (m_pixelFormat == V4L2_PIX_FMT_MJPEG ? " MJPEG" : " YUYV") << std::endl;

    return true;
}

bool V4L2VideoCapture::negotiateFormat(int width, int height)
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    V4L2_LOG_INFO("V4L2VideoCapture::negotiateFormat") << "Trying MJPEG at " << width << "x" << height;

    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        V4L2_LOG_INFO("V4L2VideoCapture::negotiateFormat") << "MJPEG failed: " << strerror(errno) << ", trying YUYV";
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
            V4L2_LOG_ERROR("V4L2VideoCapture::negotiateFormat")
            << "Neither MJPEG nor YUYV supported at " << width << "x" << height << ": " << strerror(errno);
            return false;
        }
    }

    m_pixelFormat = fmt.fmt.pix.pixelformat;
    m_width = fmt.fmt.pix.width;
    m_height = fmt.fmt.pix.height;

    V4L2_LOG_INFO("V4L2VideoCapture::negotiateFormat") << "Final format: "
    << (m_pixelFormat == V4L2_PIX_FMT_MJPEG ? "MJPEG" : "YUYV")
    << " at " << m_width << "x" << m_height;

    return true;
}

bool V4L2VideoCapture::setupBuffers()
{
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = k_bufferCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::setupBuffers") << "VIDIOC_REQBUFS failed: "
            << strerror(errno) << std::endl;
        return false;
    }

    if (req.count < 2)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::setupBuffers")
            << "Device only granted " << req.count << " buffer(s)" << std::endl;
        return false;
    }

    m_buffers.resize(req.count);

    for (unsigned int i = 0; i < req.count; ++i)
    {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(m_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::setupBuffers") << "VIDIOC_QUERYBUF failed: "
                << strerror(errno) << std::endl;
            return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset);

        if (m_buffers[i].start == MAP_FAILED)
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::setupBuffers") << "mmap failed: " << strerror(errno) << std::endl;
            m_buffers[i].start = nullptr;
            return false;
        }
    }

    return true;
}

bool V4L2VideoCapture::startStreaming()
{
    for (size_t i = 0; i < m_buffers.size(); ++i)
    {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = static_cast<unsigned int>(i);

        if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0)
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::startStreaming") << "VIDIOC_QBUF failed: "
                << strerror(errno) << std::endl;
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::startStreaming") << "VIDIOC_STREAMON failed: "
            << strerror(errno) << std::endl;
        return false;
    }

    m_streaming = true;
    return true;
}

void V4L2VideoCapture::stopStreaming()
{
    if (m_streaming && m_fd >= 0)
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    }
    m_streaming = false;
}

void V4L2VideoCapture::releaseBuffers()
{
    for (size_t i = 0; i < m_buffers.size(); ++i)
    {
        if (m_buffers[i].start != nullptr && m_buffers[i].start != MAP_FAILED)
        {
            munmap(m_buffers[i].start, m_buffers[i].length);
        }
    }
    m_buffers.clear();
}

void V4L2VideoCapture::close()
{
    stopStreaming();
    releaseBuffers();

    if (m_fd >= 0)
    {
        ::close(m_fd);
        m_fd = -1;
    }

    m_devicePath.clear();
    m_width = 0;
    m_height = 0;
}

bool V4L2VideoCapture::isOpened() const
{
    return m_fd >= 0 && m_streaming;
}

bool V4L2VideoCapture::read(cv::Mat &outFrame)
{
    outFrame.release();

    if (!isOpened())
        return false;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int selectResult = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
    if (selectResult <= 0)
    {
        if (selectResult < 0)
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::read")
            << "select() failed: " << strerror(errno);
        }

        return false;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_DQBUF, &buf) < 0)
    {
        if (errno != EAGAIN)
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::read")
            << "VIDIOC_DQBUF failed: " << strerror(errno);
        }

        return false;
    }

    bool decodeSuccess = false;
    cv::Mat decoded;

    if (buf.index >= m_buffers.size())
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::read")
        << "Invalid buffer index " << buf.index
        << " (buffer count " << m_buffers.size() << ")";

        return false;
    }

    if (buf.bytesused > 0)
    {
        const unsigned char *data =
        static_cast<const unsigned char *>(m_buffers[buf.index].start);

        if (m_pixelFormat == V4L2_PIX_FMT_MJPEG)
        {
            m_jpegScratch.assign(data, data + buf.bytesused);

            decoded = cv::imdecode(
                m_jpegScratch,
                cv::IMREAD_COLOR);

            decodeSuccess = !decoded.empty();
        }
        else if (m_pixelFormat == V4L2_PIX_FMT_YUYV)
        {
            cv::Mat yuyv(
                m_height,
                m_width,
                CV_8UC2,
                const_cast<unsigned char *>(data),
                         m_width * 2);

            cv::cvtColor(
                yuyv,
                decoded,
                cv::COLOR_YUV2BGR_YUYV);

            decodeSuccess = !decoded.empty();
        }
        else if (m_isPS3Eye)
        {
            // Preserve the existing PS3Eye Bayer path.
            cv::Mat bayer(
                m_height,
                m_width,
                CV_8UC1,
                const_cast<unsigned char *>(data),
                          m_width);

            cv::cvtColor(
                bayer,
                decoded,
                cv::COLOR_BayerGB2BGR);

            decodeSuccess = !decoded.empty();
        }
        else
        {
            V4L2_LOG_ERROR("V4L2VideoCapture::read")
            << "Unsupported pixel format: "
            << m_pixelFormat;
        }
    }
    else
    {
        V4L2_LOG_WARNING("V4L2VideoCapture::read")
        << "Dequeued buffer contains no data";
    }

    // IMPORTANT:
    // The buffer must not be returned to the driver until userspace
    // is finished reading/copying/decoding it.
    struct v4l2_buffer qbuf;
    memset(&qbuf, 0, sizeof(qbuf));
    qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.index = buf.index;

    if (xioctl(m_fd, VIDIOC_QBUF, &qbuf) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::read")
        << "VIDIOC_QBUF failed: "
        << strerror(errno);

        return false;
    }

    if (decodeSuccess && !decoded.empty())
    {
        outFrame = decoded;
        return true;
    }

    return false;
}

bool V4L2VideoCapture::isControlSupported(uint32_t controlId) const
{
    if (m_fd < 0)
    {
        return false;
    }

    struct v4l2_queryctrl query;
    std::memset(&query, 0, sizeof(query));
    query.id = controlId;

    if (xioctl(m_fd, VIDIOC_QUERYCTRL, &query) < 0)
    {
        return false;
    }

    return !(query.flags & V4L2_CTRL_FLAG_DISABLED);
}

bool V4L2VideoCapture::getControlRange(uint32_t controlId, int32_t &outMin, int32_t &outMax, int32_t &outDefault) const
{
    if (m_fd < 0)
    {
        return false;
    }

    struct v4l2_queryctrl query;
    std::memset(&query, 0, sizeof(query));
    query.id = controlId;

    if (xioctl(m_fd, VIDIOC_QUERYCTRL, &query) < 0 || (query.flags & V4L2_CTRL_FLAG_DISABLED))
    {
        return false;
    }

    outMin = query.minimum;
    outMax = query.maximum;
    outDefault = query.default_value;

    return true;
}

bool V4L2VideoCapture::setControl(uint32_t controlId, int32_t value)
{
    if (m_fd < 0)
        return false;

    int32_t min, max, def;
    if (getControlRange(controlId, min, max, def))
    {
        if (value < min)
            value = min;
        else if (value > max)
            value = max;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = controlId;
    ctrl.value = value;

    if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::setControl") << "Failed to set control 0x"
        << std::hex << controlId << std::dec << " to " << value << ": " << strerror(errno);
        return false;
    }
    return true;
}

bool V4L2VideoCapture::getControl(uint32_t controlId, int32_t &outValue) const
{
    if (m_fd < 0)
    {
        return false;
    }

    struct v4l2_control ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = controlId;

    if (xioctl(m_fd, VIDIOC_G_CTRL, &ctrl) < 0)
    {
        return false;
    }

    outValue = ctrl.value;
    return true;
}

bool V4L2VideoCapture::setResolution(int width, int height)
{
    if (!isOpened())
        return false;

    V4L2_LOG_INFO("V4L2VideoCapture::setResolution") << "Requesting " << width << "x" << height;

    int oldWidth = m_width;
    int oldHeight = m_height;
    std::string devicePath = m_devicePath;
    int fps = m_fps;

    close();

    if (open(devicePath, width, height, fps))
    {
        V4L2_LOG_INFO("V4L2VideoCapture::setResolution") << "Successfully reopened at " << m_width << "x" << m_height;

        // Force autofocus to re-lock (only for generic webcams)
        if (!m_isPS3Eye)
        {
            if (isControlSupported(V4L2_CID_FOCUS_AUTO))
            {
                setControl(V4L2_CID_FOCUS_AUTO, 0);
                setControl(V4L2_CID_FOCUS_AUTO, 1);
            }
            else if (isControlSupported(V4L2_CID_FOCUS_ABSOLUTE))
            {
                int32_t min, max, def;
                if (getControlRange(V4L2_CID_FOCUS_ABSOLUTE, min, max, def))
                {
                    setControl(V4L2_CID_FOCUS_ABSOLUTE, (min + max) / 2);
                }
            }
        }

        return true;
    }

    V4L2_LOG_ERROR("V4L2VideoCapture::setResolution") << "Failed, reverting to " << oldWidth << "x" << oldHeight;
    open(devicePath, oldWidth, oldHeight, fps);
    return false;
}

bool V4L2VideoCapture::setFramerate(int fps)
{
    if (!isOpened())
        return false;

    struct v4l2_streamparm streamParm;
    memset(&streamParm, 0, sizeof(streamParm));
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamParm.parm.capture.timeperframe.numerator = 1;
    streamParm.parm.capture.timeperframe.denominator = fps;

    if (xioctl(m_fd, VIDIOC_S_PARM, &streamParm) < 0)
    {
        V4L2_LOG_ERROR("V4L2VideoCapture::setFramerate") << "Failed to set framerate to " << fps << ": " << strerror(errno);
        return false;
    }

    // Read back the actual framerate
    struct v4l2_streamparm readParm;
    memset(&readParm, 0, sizeof(readParm));
    readParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_G_PARM, &readParm) == 0)
    {
        int actualFps = readParm.parm.capture.timeperframe.denominator /
        readParm.parm.capture.timeperframe.numerator;
        if (actualFps != fps)
        {
            V4L2_LOG_WARNING("V4L2VideoCapture::setFramerate") << "Driver set framerate to " << actualFps << " instead of " << fps;
        }
        m_fps = actualFps;
    }
    else
    {
        m_fps = fps;
    }

    return true;
}
