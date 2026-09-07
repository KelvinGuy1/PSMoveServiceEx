// -- includes -----
#include "GenericWebcamEnumerator.h"
#include "V4L2/V4L2VideoCapture.h"
#include "ServerUtility.h"
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <climits>
#include <cstdlib>

namespace
{
    const int k_ps3eye_vendor_id = 0x1415;
    const int k_ps3eye_product_id = 0x2000;

    bool getUsbVendorProduct(const std::string &devicePath, int &outVendorId, int &outProductId)
    {
        std::string videoName = devicePath.substr(devicePath.find_last_of('/') + 1);
        std::string sysDevLink = "/sys/class/video4linux/" + videoName + "/device";

        char resolved[PATH_MAX];
        if (realpath(sysDevLink.c_str(), resolved) == nullptr)
        {
            return false;
        }

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
            {
                break;
            }
            dir = dir.substr(0, lastSlash);
        }

        return false;
    }
}

GenericWebcamEnumerator::GenericWebcamEnumerator()
    : DeviceEnumerator(CommonDeviceState::PS3EYE)
    , m_device_index(0)
{
    m_deviceType = CommonDeviceState::PS3EYE;
    refreshDeviceList();
}

void GenericWebcamEnumerator::refreshDeviceList()
{
    m_devicePaths.clear();
    std::vector<std::string> allDevices = V4L2VideoCapture::enumerateDevices();

    for (const std::string &devicePath : allDevices)
    {
        m_devicePaths.push_back(devicePath);
    }
}

bool GenericWebcamEnumerator::is_valid() const
{
    return m_device_index >= 0 && static_cast<size_t>(m_device_index) < m_devicePaths.size();
}

bool GenericWebcamEnumerator::next()
{
    ++m_device_index;
    return false;
}

int GenericWebcamEnumerator::get_vendor_id() const
{
    if (!is_valid())
    {
        return -1;
    }

    int vendorId = -1;
    int productId = -1;
    getUsbVendorProduct(m_devicePaths[m_device_index], vendorId, productId);

    return vendorId;
}

int GenericWebcamEnumerator::get_product_id() const
{
    if (!is_valid())
    {
        return -1;
    }

    int vendorId = -1;
    int productId = -1;
    getUsbVendorProduct(m_devicePaths[m_device_index], vendorId, productId);

    return productId;
}

const char *GenericWebcamEnumerator::get_path() const
{
    if (!is_valid())
    {
        return nullptr;
    }

    return m_devicePaths[m_device_index].c_str();
}
