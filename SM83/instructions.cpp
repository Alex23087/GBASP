#include <cstdint>
#include <iostream>
#include "sm83.hpp"
#include "utils.hpp"

#define T_CYC(x) (x * 4)
#define CC(x, y) ((!registers.flags.Z ^ x) & ~y) || ((!registers.flags.C ^ x) & y)


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
    print_error("Invalid opcode passed to ADC");
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
        uint8_t reg_value = register_8_index_read(registers.IR & 0b0111);
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
        uint8_t reg_value = register_8_index_read(registers.IR & 0b0111);
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
    print_error("Invalid opcode passed to AND");
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
        bool condition = CC(c_l, c_m);
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
        uint8_t sub_value = register_8_index_read(registers.IR & 0b0111);

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
    // JP HL: Jump to HL
    else if (registers.IR == 0b11101001) {
        uint16_t jump_val = fetch(registers.HL);

        registers.PC = jump_val;

        return { T_CYC(1), 1, true };
    }
    // JP cc, nn: Jump (conditional)
    else if ((registers.IR >> 5) == 0b110) {
        bool c_m = registers.IR & 0b00010000;
        bool c_l = registers.IR & 0b00001000;

        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;

        bool condition = CC(c_l, c_m);
        if (condition) {
            registers.PC = nn;
            return { T_CYC(4), 3, true };
        }

        return { T_CYC(3), 3, false };
    }
    print_error("Invalid opcode passed to JP");
}

SM83::instr_ret_info SM83::JR() {
    // JR e: Relative jump
    if (registers.IR == 0b00011000) {
        uint8_t offset = fetch(registers.PC + 1);
        registers.PC += offset;
        return { T_CYC(3), 2, true };
    }

    // JR cc, e: Relative jump (conditional)
    else if ((registers.IR & 0b1110011) == 0b00100000) {
        bool c_m = registers.IR & 0b00010000;
        bool c_l = registers.IR & 0b00001000;
        uint8_t offset = fetch(registers.PC + 1);

        bool condition = CC(c_l, c_m);
        if (condition) {
            registers.PC += offset;
            return { T_CYC(3), 2, true };
        }
        return { T_CYC(2), 2, false };
    }

    print_error("Invalid opcode passed to JR");
}

SM83::instr_ret_info SM83::LD() {
    // LD r, r’: Load register (register) (0b01xxxyyy)
    if ((registers.IR >> 6) == 0b01) {
        uint8_t reg_index_x = (registers.IR >> 3) & 0b0111;
        uint8_t reg_index_y = (registers.IR) & 0b0111;
        register_8_index_write(reg_index_x, register_8_index_read(reg_index_y));
        return { T_CYC(1), 1, false };
    }

    // LD r, n: Load register (immediate)
    else if ((registers.IR & 0b11000111) == 0b00000110) {
        uint8_t reg_index = (registers.IR >> 3) & 0b0111;
        uint8_t data = fetch(registers.PC + 1);
        register_8_index_write(reg_index, data);
        return { T_CYC(2), 2, false };
    }

    // LD r, (HL): Load register (indirect HL)
    else if ((registers.IR & 0b11000111) == 0b01000110) {
        uint8_t reg_index = (registers.IR >> 3) & 0b0111;
        uint8_t data = fetch(registers.HL);
        register_8_index_write(reg_index, data);
        return { T_CYC(2), 1, false };
    }

    // LD (HL), r: Load from register (indirect HL)
    else if ((registers.IR >> 3) == 0b01110) {
        uint8_t reg_index = registers.IR & 0b0111;
        uint8_t data = register_8_index_read(reg_index);
        //TODO: Use proper bus write call
        //bus.write(registers.HL, data);
        return { T_CYC(2), 1, false };
    }

    // LD (HL), n: Load from immediate data (indirect HL)
    else if (registers.IR == 0b00110110) {
        uint8_t data = fetch(registers.PC + 1);
        //TODO: Use proper bus write call
        //bus.write(registers.HL, data);
        return { T_CYC(3), 2, false };
    }

    // LD A, (BC): Load accumulator (indirect BC)
    else if (registers.IR == 0b00001010) {
        uint8_t data = fetch(registers.BC);
        registers.A = data;
        return { T_CYC(2), 1, false };
    }

    // LD A, (DE): Load accumulator (indirect DE)
    else if (registers.IR == 0b00011010) {
        uint8_t data = fetch(registers.DE);
        registers.A = data;
        return { T_CYC(2), 1, false };
    }

    // LD (BC), A: Load from accumulator (indirect BC)
    else if (registers.IR == 0b00000010) {
        uint8_t data = registers.A;
        //TODO: Use proper bus write call
        //bus.write(registers.BC, data);
        return { T_CYC(2), 1, false };
    }

    // LD (DE), A: Load from accumulator (indirect DE)
    else if (registers.IR == 0b00010010) {
        uint8_t data = registers.A;
        //TODO: Use proper bus write call
        //bus.write(registers.DE, data);
        return { T_CYC(2), 1, false };
    }

    // LD A, (nn): Load accumulator (direct)
    else if (registers.IR == 0b11111010) {
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        uint8_t data = fetch(nn);
        registers.A = data;
        return { T_CYC(4), 3, false };
    }

    // LD (nn), A: Load from accumulator (direct)
    else if (registers.IR == 0b11101010) {
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        uint8_t data = registers.A;
        //TODO: Use proper bus write call
        //bus.write(nn, data);
        return { T_CYC(4), 3, false };
    }

    // LD A, (HL-): Load accumulator (indirect HL, decrement)
    else if (registers.IR == 0b00111010) {
        uint8_t data = fetch(registers.HL);
        registers.A = data;
        registers.HL--;
        return { T_CYC(2), 1, false };
    }

    // LD (HL-), A: Load from accumulator (indirect HL, decrement)
    else if (registers.IR == 0b00110010) {
        uint8_t data = registers.A;
        //TODO: Use proper bus write call
        //bus.write(registers.HL, data);
        registers.HL--;
        return { T_CYC(2), 1, false };
    }

    // LD A, (HL+): Load accumulator (indirect HL, increment)
    else if (registers.IR == 0b00101010) {
        uint8_t data = fetch(registers.HL);
        registers.A = data;
        registers.HL++;
        return { T_CYC(2), 1, false };
    }

    // LD (HL+), A: Load from accumulator (indirect HL, increment)
    else if (registers.IR == 0b00100010) {
        uint8_t data = registers.A;
        //TODO: Use proper bus write call
        //bus.write(registers.HL, data);
        registers.HL++;
        return { T_CYC(2), 1, false };
    }

    // LD rr, nn: Load 16-bit register / register pair
    else if ((registers.IR & 0b11001111) == 0b00000001) {
        uint8_t register_index = (registers.IR >> 4) & 0b011;
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        register_16_index_write(register_index, nn);
        return { T_CYC(3), 3, false };
    }

    // LD (nn), SP: Load from stack pointer (direct)
    else if (registers.IR == 0b00001000) {
        uint8_t lsb = fetch(registers.PC + 1);
        uint8_t msb = fetch(registers.PC + 2);
        uint16_t nn = (msb << 8) | lsb;
        uint8_t lsb_sp = LSB(registers.SP);
        uint8_t msb_sp = MSB(registers.SP);
        //TODO: Use proper bus write call
        //bus.write(nn, lsb_sp);
        nn++;
        //TODO: Use proper bus write call
        //bus.write(nn, msb_sp);
        return { T_CYC(5), 3, false };
    }

    // LD SP, HL: Load stack pointer from HL
    else if (registers.IR == 0b11111001) {
        registers.SP = registers.HL;
        return { T_CYC(2), 1, false };
    }

    // LD HL, SP+e: Load HL from adjusted stack pointer
    else if (registers.IR == 0b11111000) {
        uint8_t offset = fetch(registers.PC + 1);
        registers.HL = registers.SP + offset;

        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = CARRY_4(registers.HL, registers.SP);
        registers.flags.C = CARRY_8(registers.HL, registers.SP);

        return { T_CYC(3), 2, false };
    }

    // I want a 20th LD please 😭  (AHAHAH ONLY 19 😹🫵)

    print_error("Invalid opcode passed to LD");
}

SM83::instr_ret_info SM83::LDH() {
    // LDH A, (C): Load accumulator (indirect 0xFF00+C)
    if (registers.IR == 0b11110010) {
        uint8_t data = fetch((0xFF << 8) | registers.C);
        registers.A = data;
        return { T_CYC(2), 1, false };
    }

    // LDH (C), A: Load from accumulator (indirect 0xFF00+C)
    else if (registers.IR == 0b11100010) {
        //TODO: Use proper bus write call
        //bus.write((0xFF << 8) | registers.C, registers.A);
        return { T_CYC(2), 1, false };
    }

    // LDH A, (n): Load accumulator (direct 0xFF00+n)
    else if (registers.IR == 0b11110000) {
        uint8_t data = fetch(registers.PC + 1);
        uint8_t addr = (0xFF << 8) | data;
        registers.A = fetch(addr);
        return { T_CYC(3), 2, false };
    }

    // LDH (n), A: Load from accumulator (direct 0xFF00+n)
    else if (registers.IR == 0b11100000) {
        uint8_t data = fetch(registers.PC + 1);
        uint8_t addr = (0xFF << 8) | data;
        //TODO: Use proper bus write call
        //bus.write(addr, registers.A);
        return { T_CYC(3), 2, false };
    }

    print_error("Invalid opcode passed to LDH");
}

SM83::instr_ret_info SM83::NOP() {
    return { T_CYC(1), 1, false };
}

SM83::instr_ret_info SM83::OR() {
    // OR (HL): Bitwise OR (indirect HL)
    if (registers.IR == 0b10110110) {
        uint8_t or_value = fetch(registers.HL);

        registers.A |= or_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;
        return { T_CYC(2), 1, false };
    }
    // OR r: Bitwise OR (register)
    else if ((registers.IR >> 3) == 0b10110) {
        uint8_t or_value = register_8_index_read(registers.IR & 0b0111);

        registers.A |= or_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;
        return { T_CYC(1), 1, false };
    }
    // OR n: Bitwise OR (immediate)
    else if (registers.IR == 0b11110110) {
        uint8_t or_value = fetch(registers.PC + 1);

        registers.A |= or_value;

        registers.flags.Z = registers.A == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = 0;
        return { T_CYC(2), 2, false };
    }
    print_error("Invalid opcode passed to OR");
}

SM83::instr_ret_info SM83::POP() {
    uint8_t rr = ((registers.IR & 0b00110000) >> 4);

    uint8_t lsb = pop_stack();
    uint8_t msb = pop_stack();
    uint16_t data = (msb << 8) | lsb;

    register_16_index_write(rr, data);

    return { T_CYC(3), 1 , false };
}

SM83::instr_ret_info SM83::PREF() {
    registers.PC++;
    registers.IR = fetch(registers.PC);

    instr_ret_info ret_info = (this->*prefix_instructions[registers.IR].operation)();

    // The two following lines can be removed.
    // They're here to keep our convention of not modifying PC inside the instruction
    // (Unless it's an instruction that explicitly modifies PC)
    registers.PC--;
    ret_info.cycles += 1;

    return ret_info;
}

SM83::instr_ret_info SM83::PUSH() {
    // PUSH rr: Push to stack
    uint8_t rr = ((registers.IR & 0b00110000) >> 4);
    uint16_t data = register_16_index_read(rr);

    uint8_t msb = MSB(data);
    uint8_t lsb = LSB(data);
    push_stack(msb);
    push_stack(lsb);

    return { T_CYC(4), 1, false };
}

SM83::instr_ret_info SM83::RET() {
    // RET cc: Return from function (conditional)
    if (registers.IR >> 5 == 0b110) {
        bool c_m = registers.IR & 0b00010000;
        bool c_l = registers.IR & 0b00001000;

        uint8_t lsb = pop_stack();
        uint8_t msb = pop_stack();
        uint16_t nn = (msb << 8) | lsb;

        bool condition = CC(c_l, c_m);
        if (condition) {
            registers.PC = nn;
            return { T_CYC(5), 1, true };
        }
        return { T_CYC(2), 1, false };
    }
    // RET: Return from function
    else if (registers.IR == 0b11001001) {
        uint8_t lsb = pop_stack();
        uint8_t msb = pop_stack();
        uint16_t nn = (msb << 8) | lsb;

        registers.PC = nn;

        return { T_CYC(4), 1, true };
    }
    print_error("Invalid opcode passed to RET");
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
        uint8_t sub_value = register_8_index_read(registers.IR & 0b0111);

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
        uint8_t sub_value = register_8_index_read(registers.IR & 0b0111);

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
        uint8_t xor_value = register_8_index_read(registers.IR & 0b0111);

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
SM83::instr_ret_info SM83::BIT() {
    // BIT b, (HL): Test bit (indirect HL) (0b01xxx110)
    if ((registers.IR & 0b11000111) == 0b01000110) {
        uint8_t bit_index = (registers.IR & 0b00111000) >> 3;
        uint8_t data = fetch(registers.HL);
        uint8_t bit_mask = 1 << bit_index;
        registers.flags.Z = (data & bit_mask) == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        return { T_CYC(3), 2, false };
    }

    // BIT b, r: Test bit (register) (0b01bbbrrr)
    else if ((registers.IR & 0b11000000) == 0b01000000) {
        uint8_t bit_index = (registers.IR & 0b00111000) >> 3;
        uint8_t reg_index = registers.IR & 0b00000111;
        uint8_t data = register_8_index_read(reg_index);
        uint8_t bit_mask = 1 << bit_index;

        registers.flags.Z = (data & bit_mask) == 0;
        registers.flags.N = 0;
        registers.flags.H = 1;
        return { T_CYC(2), 2, false };
    }

    print_error("Invalid opcode passed to BIT");
}

SM83::instr_ret_info SM83::RES() {
    // RES b, r: Reset bit (register) (0b10bbbrrr)
    if ((registers.IR & 0b11000000) == 0b10000000) {
        uint8_t bit_index = (registers.IR & 0b00111000) >> 3;
        uint8_t reg_index = registers.IR & 0b00000111;
        uint8_t data = register_8_index_read(reg_index);
        uint8_t bit_mask = ~(1 << bit_index);
        register_8_index_write(reg_index, data & bit_mask);
        return { T_CYC(2), 2, false };
    }

    // RES b, (HL): Reset bit (indirect HL) (0b10bbb110)
    else if ((registers.IR & 0b11000111) == 0b10000110) {
        //TODO
    }


    print_error("Invalid opcode passed to RES");
}

SM83::instr_ret_info SM83::RL() {}
SM83::instr_ret_info SM83::RLC() {}
SM83::instr_ret_info SM83::RR() {}
SM83::instr_ret_info SM83::SET() {}
SM83::instr_ret_info SM83::SLA() {}
SM83::instr_ret_info SM83::SRA() {}
SM83::instr_ret_info SM83::SRL() {
    // SRL (HL): Shift right logical (indirect HL)
    if (registers.IR == 0b00111110) {
        uint8_t prev_value = fetch(registers.HL);
        uint8_t result = prev_value << 1 | 0b10000000;

        //TODO: Use proper bus write call
        //bus.write(registers.HL, result);

        registers.flags.Z = result == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = CARRY_8(result, prev_value);
        return { T_CYC(4), 2, false };
    }
    // SRL r: Shift right logical (register)
    else if ((registers.IR >> 3) == 0b00111) {
        uint8_t prev_value = register_8_index_read(registers.IR & 0b0111);
        uint8_t result = prev_value << 1 | 0b10000000;

        registers.flags.Z = result == 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = CARRY_8(result, prev_value);
        return { T_CYC(2), 2, false };
    }
    print_error("Invalid opcode passed to SRL");
}

SM83::instr_ret_info SM83::SWAP() {}