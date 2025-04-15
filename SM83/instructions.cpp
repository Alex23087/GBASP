#include <cstdint>
#include <iostream>
#include "sm83.hpp"
#include "utils.hpp"

#define T_CYC(x) (x * 4)

// Unprefixed instructions
SM83::instr_ret_info SM83::ADC() {
    // ADC (HL): Add with carry (indirect HL)
    if (registers.IR == 0b10001110) {
        uint8_t prev_a_value = registers.A;
        uint8_t add_value = fetch(registers.HL);
        registers.A += add_value + (registers.flags.C ? 1 : 0);

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2) , 1, false };
    }

    // ADC r: Add with carry (register)
    else if ((registers.IR >> 3) == 0b10001) {
        uint8_t reg_value = register_8_index_read(registers.IR & 0b0111);
        uint8_t prev_a_value = registers.A;

        registers.A += reg_value + (registers.flags.C ? 1 : 0);

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(1), 1, false };
    }

    // ADC n: Add with carry (immediate)
    else if (registers.IR == 0b11001110) {
        uint8_t add_value = fetch(registers.PC + 1);
        uint8_t prev_a_value = registers.A + (registers.flags.C ? 1 : 0);

        registers.A += add_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2), 2, false };
    }
}

SM83::instr_ret_info SM83::ADD() {
    // ADD (HL): Add (indirect HL)
    if (registers.IR == 0b10000110) {
        uint8_t prev_a_value = registers.A;
        uint8_t add_value = fetch(registers.HL);
        registers.A += add_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2) , 1, false };
    }

    // ADD r: Add (register)
    else if ((registers.IR >> 3) == 0b10000) {
        uint8_t reg_value = register_8_index_read(registers.IR & 0b111);
        uint8_t prev_a_value = registers.A;

        registers.A += reg_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(1), 1, false };
    }
    // ADD n: Add (immediate)
    else if (registers.IR == 0b11000110) {
        uint8_t add_value = fetch(registers.PC + 1);
        uint8_t prev_a_value = registers.A;

        registers.A += add_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2), 2, false };
    }
    // ADD SP, e: Add to stack pointer (relative)
    else if (registers.IR == 0b11101000) {
        uint16_t prev_sp_value = registers.SP;
        int8_t add_value = fetch(registers.PC + 1);
        registers.SP += add_value;

        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.SP, prev_sp_value);
        registers.flags.C = CARRY_8(registers.SP, prev_sp_value);
        return { T_CYC(4), 2, false };
    }

    print_error("Invalid ADD operation");
}

SM83::instr_ret_info SM83::AND() {
    // AND (HL): Bitwise AND (indirect HL)
    if (registers.IR == 0b10100110) {
        uint8_t and_value = fetch(registers.HL);
        registers.A &= and_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(2), 1, false };

    }
    // AND r: Bitwise AND (register)
    else if ((registers.IR >> 3) == 0b10100) {
        uint8_t reg_value = register_8_index_read(registers.IR & 0b111);
        registers.A &= reg_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(1), 1, false };
    }

    // AND n: Bitwise AND (immediate)
    else if (registers.IR == 0b11100110) {
        uint8_t and_value = fetch(registers.PC + 1);
        registers.A &= and_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        registers.flags.C = 0;

        return { T_CYC(2), 2, false };
    }
}

SM83::instr_ret_info SM83::CALL() {
    // CALL nn: Call function
    if (registers.IR == 0b11001101) {
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        push_stack(MSB(registers.PC));
        push_stack(LSB(registers.PC));
        registers.PC = nn;
        return { T_CYC(6), 3, true };
    }

    // CALL cc, nn: Call function (conditional)
    if ((registers.IR & 0b11100111) == 0b11000100) {
        bool c_m = registers.IR & 0b00010000;
        bool c_l = registers.IR & 0b00001000;
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        bool condition = ((registers.flags.Z ^ c_l) ^ c_m) || ((registers.flags.C ^ c_l) & c_m);
        if (condition) {
            push_stack(MSB(registers.PC));
            push_stack(LSB(registers.PC));
            registers.PC = nn;
            return { T_CYC(6), 3, true };
        }
        return { T_CYC(3), 3, false };
    }

    print_error("Invalid opcode passed to CALL");
}

SM83::instr_ret_info SM83::CCF() {
    registers.flags.N = false;
    registers.flags.H = false;
    registers.flags.H ^= true;
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::CP() {
    // CP (HL): Compare (indirect HL)
    if (registers.IR == 0b10011110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.HL);

        uint8_t result = registers.A - sub_value;

        registers.flags.Z = result == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, prev_a_value);
        registers.flags.C = CARRY_8(result, prev_a_value);

        return { T_CYC(2), 1, false };
    }

    // CP n: Compare (immediate)
    else if (registers.IR == 0b11111110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.PC + 1);

        uint8_t result = registers.A - sub_value;

        registers.flags.Z = result == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, prev_a_value);
        registers.flags.C = CARRY_8(result, prev_a_value);

        return { T_CYC(2), 2, false };
    }

    // CP r: Compare (register)
    else if ((registers.IR >> 3) == 0b10111) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = register_8_index_read(registers.IR & 0b111);

        uint8_t result = registers.A - sub_value;

        registers.flags.Z = result == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, prev_a_value);
        registers.flags.C = CARRY_8(result, prev_a_value);

        return { T_CYC(1), 1, false };
    }

    print_error("Invalid opcode passed to CP");
}

SM83::instr_ret_info SM83::CPL() {
    // CPL: Complement accumulator (0b00101111)
    registers.A = ~registers.A;
    registers.flags.N = true;
    registers.flags.H = true;
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::DAA() {
    // DAA: Decimal adjust accumulator (0b00100111)
    print_error("Opcode DAA not implemented");
}

SM83::instr_ret_info SM83::DEC() {
    // DEC r: Decrement (register)
    if ((registers.IR & 0b11000111) == 0b00000101) {
        // TODO: Verify that the register indexing is consistent between opcodes
        uint8_t register_index = (registers.IR >> 3) & 0b0111;
        uint8_t prev_reg_value = register_8_index_read(register_index);
        uint8_t result = register_8_index_read(register_index) - 1;
        register_8_index_write(register_index, result);

        registers.flags.Z = (result == 0 ? 1 : 0);
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, prev_reg_value);
        return { T_CYC(1), 1, false };
    }

    // DEC (HL): Decrement (indirect HL)
    else if (registers.IR == 0b00110101) {
        uint8_t data = fetch(registers.HL);
        uint8_t result = data - 1;
        //TODO: Use proper bus write call
        //bus.write(registers.HL, result);

        registers.flags.Z = (result == 0 ? 1 : 0);
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, data);
        return { T_CYC(3), 1, false };
    }

    // DEC rr: Decrement 16-bit register
    else if ((registers.IR & 0b11001111) == 0b00001011) {
        // TODO: Does this not set flags? Verify
        uint8_t register_index = (registers.IR >> 4) & 0b011;
        register_16_index_write(register_index, register_16_index_read(register_index) - 1);
        return { T_CYC(2), 1, false };
    }

    print_error("Invalid opcode passed to DEC");
}

SM83::instr_ret_info SM83::DI() {
    ASSERT(registers.IR == 0b11110011);
    // DI: Disable interrupts (0b11110011)
    registers.IME = false;
    registers.IME_DEFER = false;
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::EI() {
    ASSERT(registers.IR == 0b11111011);
    // EI: Enable interrupts (0b11111011)
    registers.IME_DEFER = true;
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::HALT() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}

SM83::instr_ret_info SM83::INC() {
    // INC r: Increment (register)
    if ((registers.IR & 0b11000111) == 0b00000100) {
        // TODO: Verify that the register indexing is consistent between opcodes
        uint8_t register_index = (registers.IR >> 3) & 0b0111;
        uint8_t prev_reg_value = register_8_index_read(register_index);
        uint8_t result = register_8_index_read(register_index) + 1;
        register_8_index_write(register_index, result);

        registers.flags.Z = (result == 0 ? 1 : 0);
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, prev_reg_value);
        return { T_CYC(1), 1, false };
    }

    // INC (HL): Increment (indirect HL)
    else if (registers.IR == 0b00110100) {
        uint8_t data = fetch(registers.HL);
        uint8_t result = data + 1;
        //TODO: Use proper bus write call
        //bus.write(registers.HL, result);

        registers.flags.Z = (result == 0 ? 1 : 0);
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(result, data);
        return { T_CYC(3), 1, false };
    }

    // INC rr: Increment 16-bit register
    else if ((registers.IR & 0b11001111) == 0b00000011) {
        // TODO: Does this not set flags? Verify
        uint8_t register_index = (registers.IR >> 4) & 0b011;
        register_16_index_write(register_index, register_16_index_read(register_index) + 1);
        return { T_CYC(2), 1, false };
    }

    print_error("Invalid opcode passed to INC");
}

SM83::instr_ret_info SM83::JP() {
    // JP nn: Jump
    if (registers.IR == 0b11000011) {
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        registers.PC = nn;
        return { T_CYC(4), 3, true };
    }
}

SM83::instr_ret_info SM83::JR() {}
SM83::instr_ret_info SM83::LD() {}
SM83::instr_ret_info SM83::LDH() {}

SM83::instr_ret_info SM83::NOP() {
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::OR() {}
SM83::instr_ret_info SM83::POP() {}
SM83::instr_ret_info SM83::PREF() {}
SM83::instr_ret_info SM83::PUSH() {
    // PUSH rr: Push to stack



    return { T_CYC(4), 1, false };
}

SM83::instr_ret_info SM83::RET() {
    // TODO UNFINISHED
    // RET cc: Return from function (conditional)
    if (registers.IR >> 5 == 0b110) {

    }
    // RET: Return from function
    else if (registers.IR == 0b11001001) {
        uint8_t lsb = pop_stack();
        uint8_t msb = pop_stack();
        uint16_t nn = (msb << 8) | lsb;

        registers.PC = nn;

        return { T_CYC(4), 1, true };
    }



}

SM83::instr_ret_info SM83::RETI() {
    // RETI: Return from interrupt handler
    uint8_t lsb = pop_stack();
    uint8_t msb = pop_stack();
    uint16_t nn = (msb << 8) | lsb;

    registers.PC = nn;
    registers.IME = true;

    return { T_CYC(4), 1, true };
}

SM83::instr_ret_info SM83::RLA() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}

SM83::instr_ret_info SM83::RLCA() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}
SM83::instr_ret_info SM83::RRA() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}

SM83::instr_ret_info SM83::RRCA() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}

SM83::instr_ret_info SM83::RST() {
    // RST n: Restart / Call function (implied) (opcode 0b11xxx111)
    uint8_t lsb = registers.IR & 0b00111000;
    uint8_t msb = 0x00;
    uint16_t nn = (msb << 8) | lsb;

    push_stack(MSB(registers.PC));
    push_stack(LSB(registers.PC));

    registers.PC = nn;

    return { T_CYC(4), 1, true };
}

SM83::instr_ret_info SM83::SBC() {
    //SBC n : Subtract with carry(immediate)
    if (registers.IR == 0b11011110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.PC + 1);

        registers.A -= sub_value - registers.flags.C;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return{ T_CYC(2), 2, false };
    }
    //SBC(HL) : Subtract with carry(indirect HL)
    else if (registers.IR == 0b10011110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.HL);

        registers.A -= sub_value - registers.flags.C;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2), 1, false };
    }
    // SBC r: Subtract with carry (register)
    else if ((registers.IR >> 3) == 0b10011) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = register_8_index_read(registers.IR & 0b111);

        registers.A -= sub_value - registers.flags.C;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return  { T_CYC(1), 1, false };
    }
    print_error("Invalid opcode passed to SBC");
}

SM83::instr_ret_info SM83::SCF() {
    // SCF: Set carry flag (opcode 0b00110111)
    registers.flags.N = 0;
    registers.flags.H = 0;
    registers.flags.C = 1;

    return{ T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::STOP() {
    print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
}

SM83::instr_ret_info SM83::SUB() {
    //SUB(HL) : Subtract(indirect HL)
    if (registers.IR == 0b10010110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.HL);

        registers.A -= sub_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(2), 1, false };
    }
    // SUB n: Subtract (immediate)
    else if (registers.IR == 0b11010110) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = fetch(registers.PC + 1);

        registers.A -= sub_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return{ T_CYC(2), 2, false };

    }
    // SUB r: Subtract (register)
    else if ((registers.IR >> 3) == 0b10010) {
        uint8_t prev_a_value = registers.A;
        uint8_t sub_value = register_8_index_read(registers.IR & 0b111);

        registers.A -= sub_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 1;
        registers.flags.H = CARRY_4(registers.A, prev_a_value);
        registers.flags.C = CARRY_8(registers.A, prev_a_value);

        return { T_CYC(1), 1, false };
    }

    print_error("Invalid opcode passed to SUB");
}

SM83::instr_ret_info SM83::XOR() {
    // XOR(HL) : Bitwise XOR(indirect HL)
    if (registers.IR == 0b10101110) {
        uint8_t prev_a_value = registers.A;
        uint8_t xor_value = fetch(registers.HL);

        registers.A ^= xor_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;

        return { T_CYC(2), 1, false };
    }
    // XOR n: Bitwise XOR (immediate)
    else if (registers.IR == 0b11101110) {
        uint8_t prev_a_value = registers.A;
        uint8_t xor_value = fetch(registers.PC + 1);

        registers.A ^= xor_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;

        return { T_CYC(2), 2, false };
    }
    // XOR r: Bitwise XOR (register)
    else if ((registers.IR >> 3) == 0b10101) {
        uint8_t prev_a_value = registers.A;
        uint8_t xor_value = register_8_index_read(registers.IR & 0b111);

        registers.A ^= xor_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;

        return { T_CYC(1), 1, false };
    }
    print_error("Invalid opcode passed to XOR");
}


SM83::instr_ret_info SM83::XXX() {
    print_error("Invalid opcode: Out of range");
}

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

