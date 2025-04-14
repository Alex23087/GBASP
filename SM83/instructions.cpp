#include <cstdint>
#include <iostream>
#include "sm83.hpp"
#include "utils.hpp"

#define T_CYC(x) (x * 4)

// Unprefixed instructions
SM83::instr_ret_info SM83::ADC() {}
SM83::instr_ret_info SM83::ADD() {
    // ADD (HL): Add (indirect HL)
    if (current_opcode == 0b10000110) {
        uint8_t prev_a_value = registers.A;
        uint8_t add_value = fetch(registers.HL);
        registers.A += add_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = ((registers.A & 0b00001111) < (prev_a_value & 0b00001111));
        registers.flags.C = (registers.A < prev_a_value);

        return { T_CYC(2) , 1 };
    }

    // ADD r: Add (register)
    else if ((current_opcode >> 3) == 0b10000) {
        uint8_t reg_value = register_from_index(current_opcode & 0b111);
        uint8_t prev_a_value = registers.A;

        registers.A += reg_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = ((registers.A & 0x0F) < (prev_a_value & 0x0F));
        registers.flags.C = (registers.A < prev_a_value);

        return { T_CYC(1), 1 };
    }
    // ADD n: Add (immediate)
    else if (current_opcode == 0b11000110) {
        uint8_t add_value = fetch(registers.PC + 1);
        uint8_t prev_a_value = registers.A;

        registers.A += add_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = ((registers.A & 0x0F) < (prev_a_value & 0x0F));
        registers.flags.C = (registers.A < prev_a_value);

        return { T_CYC(2), 2 };
    }
    // ADD SP, e: Add to stack pointer (relative)
    else if (current_opcode == 0b11101000) {
        uint16_t prev_sp_value = registers.SP;
        int8_t add_value = fetch(registers.PC + 1);
        registers.SP += add_value;

        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = ((registers.SP & 0x0F) < (prev_sp_value & 0x0F));
        registers.flags.C = ((registers.SP & 0x00FF) < (prev_sp_value & 0x00FF));
        return { T_CYC(4), 2 };
    }

    print_error("Invalid ADD operation");
}

SM83::instr_ret_info SM83::AND() {
    // AND (HL): Bitwise AND (indirect HL)
    if (current_opcode == 0b10100110) {
        uint8_t and_value = fetch(registers.HL);
        registers.A &= and_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(2), 1 };

    }
    // AND r: Bitwise AND (register)
    else if ((current_opcode >> 3) == 0b10100) {
        uint8_t reg_value = register_from_index(current_opcode & 0b111);
        registers.A &= reg_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(1), 1 };
    }

    // AND n: Bitwise AND (immediate)
    else if (current_opcode == 0b11100110) {
        uint8_t and_value = fetch(registers.PC + 1);
        registers.A &= and_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(2), 2 };
    }
}

SM83::instr_ret_info SM83::CALL() {}
SM83::instr_ret_info SM83::CCF() {}
SM83::instr_ret_info SM83::CP() {}
SM83::instr_ret_info SM83::CPL() {}
SM83::instr_ret_info SM83::DAA() {}
SM83::instr_ret_info SM83::DEC() {}
SM83::instr_ret_info SM83::DI() {}
SM83::instr_ret_info SM83::EI() {}
SM83::instr_ret_info SM83::HALT() {}
SM83::instr_ret_info SM83::INC() {}
SM83::instr_ret_info SM83::JP() {}
SM83::instr_ret_info SM83::JR() {}
SM83::instr_ret_info SM83::LD() {}
SM83::instr_ret_info SM83::LDH() {}
SM83::instr_ret_info SM83::NOP() {}
SM83::instr_ret_info SM83::OR() {}
SM83::instr_ret_info SM83::POP() {}
SM83::instr_ret_info SM83::PREF() {}
SM83::instr_ret_info SM83::PUSH() {}
SM83::instr_ret_info SM83::RET() {}
SM83::instr_ret_info SM83::RETI() {}
SM83::instr_ret_info SM83::RLA() {}
SM83::instr_ret_info SM83::RLCA() {}
SM83::instr_ret_info SM83::RRA() {}
SM83::instr_ret_info SM83::RRCA() {}
SM83::instr_ret_info SM83::RST() {}
SM83::instr_ret_info SM83::SBC() {}
SM83::instr_ret_info SM83::SCF() {}
SM83::instr_ret_info SM83::STOP() {}
SM83::instr_ret_info SM83::SUB() {}
SM83::instr_ret_info SM83::XOR() {}
SM83::instr_ret_info SM83::XXX() {}

// Prefixed instructions
SM83::instr_ret_info SM83::BIT() {}
SM83::instr_ret_info SM83::RES() {}
SM83::instr_ret_info SM83::RL() {}
SM83::instr_ret_info SM83::RLC() {}
SM83::instr_ret_info SM83::RR() {}
SM83::instr_ret_info SM83::RRC() {}
SM83::instr_ret_info SM83::SET() {}
SM83::instr_ret_info SM83::SLA() {}
SM83::instr_ret_info SM83::SRA() {}
SM83::instr_ret_info SM83::SRL() {}
SM83::instr_ret_info SM83::SWAP() {}


