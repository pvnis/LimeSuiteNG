#include "DeviceFactoryLiteXM2SDR.h"
#include "LiteXM2SDR/LiteXM2SDR.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/Logger.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>
#include <string>

using namespace std::literals::string_literals;

namespace lime {

DeviceFactoryLiteXM2SDR::DeviceFactoryLiteXM2SDR()
    : DeviceRegistryEntry("LiteXM2SDR"s)
{
}

std::vector<DeviceHandle> DeviceFactoryLiteXM2SDR::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;

    if (!hint.media.empty() && hint.media != "PCIe"s)
        return handles;

    for (int i = 0; i < 8; ++i)
    {
        std::string devPath = "/dev/m2sdr"s + std::to_string(i);

        struct stat st;
        if (stat(devPath.c_str(), &st) != 0)
            continue;

        int fd = open(devPath.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;

        DeviceHandle handle;
        handle.media = "PCIe"s;
        handle.name = "LiteXM2SDR"s;
        handle.addr = devPath;

        // Read DNA serial number
        handle.serial = LiteXM2SDR::ReadSerial(fd);

        close(fd);

        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }

    return handles;
}

SDRDevice* DeviceFactoryLiteXM2SDR::make(const DeviceHandle& handle)
{
    if (handle.addr.empty())
    {
        lime::ReportError(OpStatus::InvalidValue, "LiteXM2SDR: device path is empty");
        return nullptr;
    }

    try
    {
        return new LiteXM2SDR(handle.addr);
    }
    catch (const std::exception& ex)
    {
        lime::ReportError(OpStatus::Error, "LiteXM2SDR: failed to open device: %s", ex.what());
        return nullptr;
    }
}

void __loadLiteXM2SDR()
{
    static DeviceFactoryLiteXM2SDR litexM2SDRSupport;
}

} // namespace lime
