#pragma once

#include <vector>
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

#define FLAG_REGISTER(x)\
    union { \
        struct { \
            bool   : 4; \
            bool Z : 1; \
            bool N : 1; \
            bool H : 1; \
            bool C : 1; \
        } flags; \
        uint8_t x; \
    }

#define INTERRUPT_REGISTER(x)\
    union { \
        struct { \
            bool        : 3; \
            bool x##_JOYPAD : 1; \
            bool x##_SERIAL : 1; \
            bool x##_TIMER  : 1; \
            bool x##_LCDC   : 1; \
            bool x##_VBLANK : 1; \
        }; \
        uint8_t x; \
    }

typedef struct Registers {
    uint8_t IR;                 // Instruction Register
    INTERRUPT_REGISTER(IE);     // Interrupt Enable
    uint8_t A;                  // Accumulator
    FLAG_REGISTER(F);           // Flags
    REGISTER_PAIR(B, C);
    REGISTER_PAIR(D, E);
    REGISTER_PAIR(H, L);
    uint16_t PC;                // Program Counter
    uint16_t SP;                // Stack Pointer
    INTERRUPT_REGISTER(IF);     // Interrupt Flag
    bool IME : 1;               // Interrupt Master Enable
    bool IME_DEFER : 1;         // Interrupt Master Enable deferred
} Registers;

class SM83 : public Device {
public:
    typedef struct instr_ret_info { uint8_t cycles; uint8_t pc_incr; bool has_set_PC; } instr_ret_info;
    typedef instr_ret_info(SM83::* instruction_ptr)(void);

    struct Instruction {
        std::string name;
        instruction_ptr operation = nullptr;
    };

    SM83();
    ~SM83();
    void run();
    void reset();
    void fill_instruction_array();
    void write(uint16_t address, uint8_t data) override;
    uint8_t read(uint16_t address) override;
private:
    Registers registers;

    uint32_t cycles;
    uint8_t fetch(uint16_t);
    void decode_execute(uint8_t opcode);
    std::vector<Instruction> instructions;
    std::vector<Instruction> prefix_instructions;

    void push_stack(uint8_t data);
    uint8_t pop_stack();

    // Utility functions
    uint8_t register_8_index_read(uint8_t index);
    void register_8_index_write(uint8_t index, uint8_t data);
    uint16_t register_16_index_read(uint8_t index);
    void register_16_index_write(uint8_t index, uint16_t data);

    // Unprefixed instructions
    instr_ret_info ADC(); instr_ret_info ADD(); instr_ret_info AND(); instr_ret_info CALL();
    instr_ret_info CCF(); instr_ret_info CP(); instr_ret_info CPL(); instr_ret_info DAA();
    instr_ret_info DEC(); instr_ret_info DI(); instr_ret_info EI(); instr_ret_info HALT();
    instr_ret_info INC(); instr_ret_info JP(); instr_ret_info JR(); instr_ret_info LD();
    instr_ret_info LDH(); instr_ret_info NOP(); instr_ret_info OR(); instr_ret_info POP();
    instr_ret_info PREF(); instr_ret_info PUSH(); instr_ret_info RET(); instr_ret_info RETI();
    instr_ret_info RLA(); instr_ret_info RLCA(); instr_ret_info RRA(); instr_ret_info RRCA();
    instr_ret_info RST(); instr_ret_info SBC(); instr_ret_info SCF(); instr_ret_info STOP();
    instr_ret_info SUB(); instr_ret_info XOR(); instr_ret_info XXX();
    // Prefixed instructions
    instr_ret_info BIT(); instr_ret_info RES(); instr_ret_info RL(); instr_ret_info RLC();
    instr_ret_info RR(); instr_ret_info RRC(); instr_ret_info SET(); instr_ret_info SLA();
    instr_ret_info SRA(); instr_ret_info SRL(); instr_ret_info SWAP();
};