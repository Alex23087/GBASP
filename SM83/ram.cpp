#include <cstdint>
#include <array>
#include "ram.hpp"
#include "device.hpp"

using namespace std;

Ram::Ram() {};
Ram::~Ram() {};

void Ram::write(uint16_t address, uint8_t data) {
    memory.at(address) = data;
}

uint8_t Ram::read(uint16_t address) {
    return memory.at(address);
}