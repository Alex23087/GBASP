#pragma once

#include <cstdint>
#include <array>
#include "device.hpp"

using namespace std;

class RAM : public Device {
public:
    RAM();
    ~RAM();
    void write(uint16_t address, uint8_t data);
    uint8_t read(uint16_t address);

private:
    std::array<uint8_t, 64 * 1024> memory = { 0 };
};