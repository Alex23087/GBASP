#pragma once

#include <cstdint>

class Device {
public:
    virtual void write(uint16_t address, uint8_t data) = 0;
    virtual uint8_t read(uint16_t address) = 0;
    virtual void writeWord(uint16_t address, uint16_t data) = 0;
    virtual uint16_t readWord(uint16_t address) = 0;
};