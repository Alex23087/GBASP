#pragma once

#include <cstdint>
#include <vector>
#include "device.hpp"

class Bus {
public:
    Bus();
    ~Bus();
    void write(uint16_t address, uint8_t data);
    uint8_t read(uint16_t address);
    void attach_device(uint16_t first_address, uint16_t last_address, Device* device);

private:
    typedef struct DeviceMapping {
        uint16_t first_address;
        uint16_t last_address;
        Device* device;
    } DeviceMapping;

    std::vector<DeviceMapping> devices;
    DeviceMapping* find_device(uint16_t address);
};