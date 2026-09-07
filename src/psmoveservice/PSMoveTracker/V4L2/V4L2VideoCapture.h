#ifndef V4L2_VIDEO_CAPTURE_H
#define V4L2_VIDEO_CAPTURE_H

// -- includes -----
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <cstdint>

/// Minimal V4L2-backed video capture for generic UVC webcams on Linux.
/// Captures MJPEG frames directly from the kernel via mmap'd buffers and
/// decodes them with cv::imdecode(), sidestepping GStreamer/OpenCV's
/// VideoCapture property-negotiation quirks (live property changes crashing
/// the pipeline, unsupported/"unhandled property" warnings, etc.) entirely.
///
/// Deliberately NOT a cv::VideoCapture subclass: this is a fresh, minimal
/// implementation for the generic-webcam path only. PS3Eye/CLEye/Virtual
/// tracker support continues to use PSEyeVideoCapture unchanged.
class V4L2VideoCapture
{
public:
    V4L2VideoCapture();
    ~V4L2VideoCapture();

    /// Opens /dev/video<index> (or a specific device path) and negotiates
    /// MJPEG capture at the requested resolution. Returns false on any
    /// failure (device missing, format unsupported, buffer setup failed).
    bool open(int index, int width = 640, int height = 480, int fps = 30);
    bool open(const std::string &devicePath, int width = 640, int height = 480, int fps = 30);

    void close();
    bool isOpened() const;

    /// Blocks until a frame is available, decodes it, and stores it in
    /// outFrame as a BGR cv::Mat. Returns false on any failure (device
    /// error, empty/corrupt JPEG, capture stopped). Mirrors the semantics
    /// of cv::VideoCapture::read() so call sites can treat it identically.
    bool read(cv::Mat &outFrame);

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    int getFps() const { return m_fps; }
    const std::string &getDevicePath() const { return m_devicePath; }

    /// Real V4L2 control access (VIDIOC_QUERYCTRL / VIDIOC_S_CTRL /
    /// VIDIOC_G_CTRL), not property emulation. controlId is a V4L2_CID_*
    /// constant (see <linux/v4l2-controls.h>), e.g. V4L2_CID_EXPOSURE_ABSOLUTE.
    bool isControlSupported(uint32_t controlId) const;
    bool getControlRange(uint32_t controlId, int32_t &outMin, int32_t &outMax, int32_t &outDefault) const;
    bool setControl(uint32_t controlId, int32_t value);
    bool getControl(uint32_t controlId, int32_t &outValue) const;
    bool setResolution(int width, int height);
    bool setFramerate(int fps);

    std::string getPersistentIdentifier() const;
    /// Enumerates capture-capable /dev/video* device paths on the system.
    static std::vector<std::string> enumerateDevices();

    int getIndex() const { return m_index; }

private:
    struct MappedBuffer
    {
        void *start;
        size_t length;
    };

    bool openDevice(const std::string &devicePath, int width, int height, int fps);
    bool negotiateFormat(int width, int height);
    bool setupBuffers();
    bool startStreaming();
    void stopStreaming();
    void releaseBuffers();

    int m_fd;
    std::string m_devicePath;
    int m_pixelFormat; // V4L2_PIX_FMT_MJPEG or V4L2_PIX_FMT_YUYV
    int m_width;
    int m_height;
    int m_fps;
    bool m_streaming;
    bool m_isPS3Eye;
    int m_index = -1;
    std::vector<MappedBuffer> m_buffers;
    std::vector<unsigned char> m_jpegScratch; // reused decode scratch buffer

};

#endif // V4L2_VIDEO_CAPTURE_H
