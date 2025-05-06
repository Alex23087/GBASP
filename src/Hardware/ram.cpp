#include <cstdint>
#include <array>
#include "ram.hpp"
#include "device.hpp"

using namespace std;

RAM::RAM() {
    printf("INITIALIZING RAM\n");
};
RAM::~RAM() {};

void RAM::write(uint16_t address, uint8_t data) {
    memory.at(address) = data;
}

uint8_t RAM::read(uint16_t address) {
    return memory.at(address);
}