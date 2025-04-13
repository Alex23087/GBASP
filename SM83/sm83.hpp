#pragma once

#include "device.hpp"
#include <cstdint>

#define REGISTER_PAIR(x,y) \
    union { \
        struct { \
            uint8_t x; \
            uint8_t y; \
        }; \
        uint16_t x##y; \
    }

typedef struct Registers {
    uint8_t IR;
    uint8_t IE;
    REGISTER_PAIR(A, F);
    REGISTER_PAIR(B, C);
    REGISTER_PAIR(D, E);
    REGISTER_PAIR(H, L);
    uint16_t PC;
    uint16_t SP;

} Registers;

class SM83 : public Device {
public:
    SM83();
    ~SM83();
    void run();
    void reset();
    void write(uint16_t address, uint8_t data) override;
    uint8_t read(uint16_t address) override;
    void writeWord(uint16_t address, uint16_t data) override;
    uint16_t readWord(uint16_t address) override;
private:
    Registers registers;
    uint32_t cycles;
    void fetch();
    void decode_execute();
};