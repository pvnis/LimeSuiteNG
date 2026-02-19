#pragma once

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceFactoryLiteXM2SDR : public DeviceRegistryEntry
{
  public:
    DeviceFactoryLiteXM2SDR();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime
