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
    uint8_t out = memory.at(address);
#if GBASP_DEBUG
    printf("[WRAM] Read %02X from address %04X\n", out, address);
#endif
    return out;
}