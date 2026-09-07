#ifndef GENERIC_WEBCAM_ENUMERATOR_H
#define GENERIC_WEBCAM_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"
#include <vector>
#include <string>

// -- definitions -----
/// Enumerates generic UVC webcams (any /dev/video* device that isn't a
/// known PS3Eye-family camera) for use with V4L2VideoCapture. Lets any
/// webcam or PS4 camera show up as a tracker without hardcoding a specific
/// USB vendor/product ID.
class GenericWebcamEnumerator : public DeviceEnumerator
{
public:
    GenericWebcamEnumerator();

    bool is_valid() const override;
    bool next() override;
    int get_vendor_id() const override;
    int get_product_id() const override;
    const char *get_path() const override;

    inline int get_device_identifier() const { return m_device_index; }

private:
    /// Populates m_devicePaths with every /dev/video* device that supports
    /// streaming capture and is NOT a recognized PS3Eye-family camera
    /// (checked via sysfs USB vendor/product ID, to avoid double-listing a
    /// real PS3Eye as both itself and a generic webcam).
    void refreshDeviceList();

    std::vector<std::string> m_devicePaths;
    int m_device_index;
};

#endif // GENERIC_WEBCAM_ENUMERATOR_H
