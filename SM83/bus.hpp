#pragma once

#include <cstdint>
#include "device.hpp"

class Bus {
public:
    Bus();
    ~Bus();
    void write(uint16_t address, uint8_t data);
    uint8_t read(uint16_t address);
    void writeWord(uint16_t address, uint16_t data);
    uint16_t readWord(uint16_t address);
    void attachDevice(uint16_t start, uint16_t end, Device* device);
};