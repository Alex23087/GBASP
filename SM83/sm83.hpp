#pragma once

#include <array>
#include <cstdint>
#include <string>
#include "device.hpp"

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
    struct Instruction {
        std::string name;
        uint8_t(SM83::* operate)(void) = nullptr;
        uint8_t cycles = 0;
    };

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
    void decode_execute(uint8_t opcode);
    std::array<Instruction, 256> instructions;
    std::array<Instruction, 256> prefix_instructions;

    // Unprefixed instructions
    void ADC(); void ADD(); void AND(); void CALL();
    void CCF(); void CP(); void CPL(); void DAA();
    void DEC(); void DI(); void EI(); void HALT();
    void INC(); void JP(); void JR(); void LD();
    void LDH(); void NOP(); void OR(); void POP();
    void PREF(); void PUSH(); void RET(); void RETI();
    void RLA(); void RLCA(); void RRA(); void RRCA();
    void RST(); void SBC(); void SCF(); void STOP();
    void SUB(); void XOR(); void XXX();
    // Prefixed instructions
    void BIT(); void RES(); void RL(); void RLC();
    void RR(); void RRC(); void SET(); void SLA();
    void SRA(); void SRL(); void SWAP();
};