#include "utils.hpp"

#include <string>
#include <vector>

namespace Parser {
    typedef struct parse_instr_info { std::string operation_pp; uint8_t pc_incr; } parse_instr_info;
    typedef parse_instr_info(function_ptr)(uint8_t[]);
    std::vector<function_ptr> instructions;


    std::string register_8_to_register_name(uint8_t index) {
        switch (index) {
            case 0:
                return "B";
            case 1:
                return "C";
            case 2:
                return "D";
            case 3:
                return "E";
            case 4:
                return "H";
            case 5:
                return "L";
            case 7:
                return "A";
            default:
                print_error("Invalid index passed to register_8_to_register_name");
        }
    }

    std::string register_16_to_register_name(uint8_t index) {
        switch (index) {
            case 0:
                return "BC";
            case 1:// TODO: Add proper bus read/write function calls
                // 
            case 3:
                return "SP";
            default:
                print_error("Invalid index passed register_16_index_read");
        }
    }

    std::string CC(bool c_l, bool c_m) {
        return "??????";
    }


    parse_instr_info parse_instruction(uint8_t opcode[]) {
        return (*instructions[opcode[0]])(opcode);
    }

    // Unprefixed instructions
    parse_instr_info ADC(uint8_t opcode[]) {
        // ADC (HL): Add with carry (indirect HL)
        if (opcode[0] == 0b10001110) {
            return { "ADC [HL]", 1 };
        }

        // ADC r: Add with carry (register)
        else if ((opcode[0] >> 3) == 0b10001) {
            std::string reg_name = register_8_to_register_name(opcode[0] & 0b0111);
            return { "ADC " + reg_name, 1 };
        }

        // ADC n: Add with carry (immediate)
        else if (opcode[0] == 0b11001110) {
            uint8_t add_value = opcode[1];
            return { "ADC " + add_value, 2 };
        }
        print_error("Invalid opcode passed to ADC");
    }

    parse_instr_info ADD(uint8_t opcode[]) {
        // ADD (HL): Add (indirect HL)
        if (opcode[0] == 0b10000110) {
            return { "ADD [HL]" , 1 };
        }

        // ADD r: Add (register)
        else if ((opcode[1] >> 3) == 0b10000) {
            std::string reg_name = register_8_to_register_name(opcode[0] & 0b0111);
            return { "ADD " + reg_name, 1 };
        }

        // ADD n: Add (immediate)
        else if (opcode[0] == 0b11000110) {
            uint8_t add_value = opcode[1];
            return { "ADD " + std::to_string(add_value), 2 };
        }

        // ADD SP, e: Add to stack pointer (relative)
        else if (opcode[0] == 0b11101000) {
            int8_t add_value = opcode[1];
            return { "ADD SP, " + std::to_string(add_value), 2 };
        }

        print_error("Invalid ADD operation");
    }

    parse_instr_info AND(uint8_t opcode[]) {
        // AND (HL): Bitwise AND (indirect HL)
        if (opcode[0] == 0b10100110) {
            return { "AND [HL]", 1 };
        }

        // AND r: Bitwise AND (register)
        else if ((opcode[0] >> 3) == 0b10100) {
            std::string reg_name = register_8_to_register_name(opcode[0] & 0b0111);
            return { "AND " + reg_name, 1 };
        }

        // AND n: Bitwise AND (immediate)
        else if (opcode[0] == 0b11100110) {
            uint8_t and_value = opcode[1];
            return { "AND " + std::to_string(and_value), 2 };
        }
        print_error("Invalid opcode passed to AND");
    }

    parse_instr_info CALL(uint8_t opcode[]) {
        // CALL nn: Call function
        if (opcode[0] == 0b11001101) {
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            return { "CALL " + std::to_string(nn), 3 };
        }

        // CALL cc, nn: Call function (conditional)
        if ((opcode[0] & 0b11100111) == 0b11000100) {
            bool c_m = opcode[0] & 0b00010000;
            bool c_l = opcode[0] & 0b00001000;
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            std::string condition = CC(c_l, c_m);
            return { "CALL " + condition + " " + std::to_string(nn) , 3 };
        }

        print_error("Invalid opcode passed to CALL");
    }

    parse_instr_info CCF(uint8_t opcode[]) {
        return { "CCF", 1 };
    }

    parse_instr_info CP(uint8_t opcode[]) {
        // CP (HL): Compare (indirect HL)
        if (opcode[0] == 0b10011110) {
            return { "CP [HL]" , 1 };
        }

        // CP n: Compare (immediate)
        else if (opcode[0] == 0b11111110) {
            uint8_t sub_value = opcode[1];
            return { "CP " + std::to_string(sub_value) , 2 };
        }

        // CP r: Compare (register)
        else if ((opcode[0] >> 3) == 0b10111) {
            std::string sub_value = register_8_to_register_name(opcode[0] & 0b0111);
            return { "CP " + sub_value, 1 };
        }

        print_error("Invalid opcode passed to CP");
    }

    parse_instr_info CPL(uint8_t opcode[]) {
        // CPL: Complement accumulator (0b00101111)
        return { "CP" , 1 };
    }

    parse_instr_info DAA(uint8_t opcode[]) {
        // DAA: Decimal adjust accumulator (0b00100111)
        print_error("opcode DAA not implemented");
    }

    parse_instr_info DEC(uint8_t opcode[]) {
        // DEC r: Decrement (register)
        if ((opcode[0] & 0b11000111) == 0b00000101) {
            // TODO: Verify that the register indexing is consistent between opcodes
            uint8_t register_index = (opcode[0] >> 3) & 0b0111;
            std::string reg_name = register_8_to_register_name(register_index);
            return { "DEC " + reg_name , 1 };
        }

        // DEC (HL): Decrement (indirect HL)
        else if (opcode[0] == 0b00110101) {
            return { "DEC [HL]" , 1 };
        }

        // DEC rr: Decrement 16-bit register
        else if ((opcode[0] & 0b11001111) == 0b00001011) {
            // TODO: Does this not set flags? Verify
            uint8_t register_index = (opcode[0] >> 4) & 0b011;
            std::string reg_name = register_16_to_register_name(register_index);
            return { "DEC " + reg_name , 1 };
        }

        print_error("Invalid opcode passed to DEC");
    }

    parse_instr_info DI(uint8_t opcode[]) {
        ASSERT(opcode[0] == 0b11110011);
        // DI: Disable interrupts (0b11110011)
        return { "DI" , 1 };
    }

    parse_instr_info EI(uint8_t opcode[]) {
        ASSERT(opcode[0] == 0b11111011);
        // EI: Enable interrupts (0b11111011)
        return { "EI" , 1 };
    }

    parse_instr_info HALT(uint8_t opcode[]) {
        return { "HALT" , 1 };
    }

    parse_instr_info INC(uint8_t opcode[]) {
        // INC r: Increment (register)
        if ((opcode[0] & 0b11000111) == 0b00000100) {
            // TODO: Verify that the register indexing is consistent between opcodes
            uint8_t register_index = (opcode[0] >> 3) & 0b0111;
            std::string reg_name = register_8_to_register_name(register_index);
            return { "INC " + reg_name , 1 };
        }

        // INC (HL): Increment (indirect HL)
        else if (opcode[0] == 0b00110100) {
            return { "INC [HL]" , 1 };
        }

        // INC rr: Increment 16-bit register
        else if ((opcode[0] & 0b11001111) == 0b00000011) {
            uint8_t register_index = (opcode[0] >> 4) & 0b011;
            std::string reg_name = register_16_to_register_name(register_index);
            return { "INC " + reg_name, 1 };
        }

        print_error("Invalid opcode passed to INC");
    }

    parse_instr_info JP(uint8_t opcode[]) {
        // JP nn: Jump
        if (opcode[0] == 0b11000011) {
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            return { "JP " + std::to_string(nn), 3 };
        }

        // JP HL: Jump to HL
        else if (opcode[0] == 0b11101001) {
            return { "JP [HL]" , 1 };
        }

        // JP cc, nn: Jump (conditional)
        else if ((opcode[0] >> 5) == 0b110) {
            bool c_m = opcode[0] & 0b00010000;
            bool c_l = opcode[0] & 0b00001000;

            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;

            std::string condition = CC(c_l, c_m);
            return { "JP " + condition + " " + std::to_string(nn) , 3 };
        }
        print_error("Invalid opcode passed to JP");
    }

    parse_instr_info JR(uint8_t opcode[]) {
        // JR e: Relative jump
        if (opcode[0] == 0b00011000) {
            uint8_t offset = opcode[1];
            return { "JR " + std::to_string(offset) , 2 };
        }

        // JR cc, e: Relative jump (conditional)
        else if ((opcode[0] & 0b1110011) == 0b00100000) {
            bool c_m = opcode[0] & 0b00010000;
            bool c_l = opcode[0] & 0b00001000;
            uint8_t offset = opcode[1];

            std::string condition = CC(c_l, c_m);
            return { "JR " + condition + " " + std::to_string(offset) , 2 };
        }

        print_error("Invalid opcode passed to JR");
    }

    parse_instr_info LD(uint8_t opcode[]) {
        // LD r, r’: Load register (register) (0b01xxxyyy)
        if ((opcode[0] >> 6) == 0b01) {
            uint8_t reg_index_x = (opcode[0] >> 3) & 0b0111;
            uint8_t reg_index_y = (opcode[0]) & 0b0111;
            std::string reg_name_x = register_8_to_register_name(reg_index_x);
            std::string reg_name_y = register_8_to_register_name(reg_index_y);
            return { "LD " + reg_name_x + " " + reg_name_y, 1 };
        }

        // LD r, n: Load register (immediate)
        else if ((opcode[0] & 0b11000111) == 0b00000110) {
            uint8_t reg_index = (opcode[0] >> 3) & 0b0111;
            uint8_t data = opcode[1];
            std::string reg_name = register_8_to_register_name(reg_index);
            return { "LD " + reg_name + " " + std::to_string(data), 2 };
        }

        // LD r, (HL): Load register (indirect HL)
        else if ((opcode[0] & 0b11000111) == 0b01000110) {
            uint8_t reg_index = (opcode[0] >> 3) & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);
            return { "LD " + reg_name + " [HL]", 1 };
        }

        // LD (HL), r: Load from register (indirect HL)
        else if ((opcode[0] >> 3) == 0b01110) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "LD [HL] " + reg_name, 1 };
        }

        // LD (HL), n: Load from immediate data (indirect HL)
        else if (opcode[0] == 0b00110110) {
            uint8_t data = opcode[1];
            return { "LD [HL] " + std::to_string(data) , 2 };
        }

        // LD A, (BC): Load accumulator (indirect BC)
        else if (opcode[0] == 0b00001010) {
            return { "LD A [BC]" , 1 };
        }

        // LD A, (DE): Load accumulator (indirect DE)
        else if (opcode[0] == 0b00011010) {
            return { "LD A [DE]" , 1 };
        }

        // LD (BC), A: Load from accumulator (indirect BC)
        else if (opcode[0] == 0b00000010) {
            return { "LD [BC] A" , 1 };
        }

        // LD (DE), A: Load from accumulator (indirect DE)
        else if (opcode[0] == 0b00010010) {
            return { "LD [DE] A" , 1 };
        }

        // LD A, (nn): Load accumulator (direct)
        else if (opcode[0] == 0b11111010) {
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            uint8_t data = fetch(nn);
            registers.A = data;
            return { , 3 };
        }

        // LD (nn), A: Load from accumulator (direct)
        else if (opcode[0] == 0b11101010) {
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            uint8_t data = registers.A;

            bus->write(nn, data);

            return { , 3 };
        }

        // LD A, (HL-): Load accumulator (indirect HL, decrement)
        else if (opcode[0] == 0b00111010) {
            uint8_t data = fetch(registers.HL);
            registers.A = data;
            registers.HL--;
            return { , 1 };
        }

        // LD (HL-), A: Load from accumulator (indirect HL, decrement)
        else if (opcode[0] == 0b00110010) {
            uint8_t data = registers.A;
            bus->write(registers.HL, data);
            registers.HL--;
            return { , 1 };
        }

        // LD A, (HL+): Load accumulator (indirect HL, increment)
        else if (opcode[0] == 0b00101010) {
            uint8_t data = fetch(registers.HL);
            registers.A = data;
            registers.HL++;
            return { , 1 };
        }

        // LD (HL+), A: Load from accumulator (indirect HL, increment)
        else if (opcode[0] == 0b00100010) {
            uint8_t data = registers.A;

            bus->write(registers.HL, data);
            registers.HL++;
            return { , 1 };
        }

        // LD rr, nn: Load 16-bit register / register pair
        else if ((opcode[0] & 0b11001111) == 0b00000001) {
            uint8_t register_index = (opcode[0] >> 4) & 0b011;
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            register_16_index_write(register_index, nn);
            return { , 3 };
        }

        // LD (nn), SP: Load from stack pointer (direct)
        else if (opcode[0] == 0b00001000) {
            uint8_t lsb = opcode[1];
            uint8_t msb = opcode[2];
            uint16_t nn = (msb << 8) | lsb;
            uint8_t lsb_sp = LSB(registers.SP);
            uint8_t msb_sp = MSB(registers.SP);

            bus->write(nn, lsb_sp);
            nn++;

            bus->write(nn, msb_sp);
            return { , 3 };
        }

        // LD SP, HL: Load stack pointer from HL
        else if (opcode[0] == 0b11111001) {
            registers.SP = registers.HL;
            return { , 1 };
        }

        // LD HL, SP+e: Load HL from adjusted stack pointer
        else if (opcode[0] == 0b11111000) {
            uint8_t offset = opcode[1];
            registers.HL = registers.SP + offset;

            registers.flags.Z = 0;
            registers.flags.N = 0;
            registers.flags.H = CARRY_4(registers.HL, registers.SP);
            registers.flags.C = CARRY_8(registers.HL, registers.SP);

            return { , 2 };
        }

        // I want a 20th LD please 😭  (AHAHAH ONLY 19 😹🫵)

        print_error("Invalid opcode passed to LD");
    }

    parse_instr_info LDH(uint8_t opcode[]) {
        // LDH A, (C): Load accumulator (indirect 0xFF00+C)
        if (opcode[0] == 0b11110010) {
            uint8_t data = fetch((0xFF << 8) | registers.C);
            registers.A = data;
            return { , 1 };
        }

        // LDH (C), A: Load from accumulator (indirect 0xFF00+C)
        else if (opcode[0] == 0b11100010) {

            bus->write((0xFF << 8) | registers.C, registers.A);
            return { , 1 };
        }

        // LDH A, (n): Load accumulator (direct 0xFF00+n)
        else if (opcode[0] == 0b11110000) {
            uint8_t data = opcode[1];
            uint8_t addr = (0xFF << 8) | data;
            registers.A = fetch(addr);
            return { , 2 };
        }

        // LDH (n), A: Load from accumulator (direct 0xFF00+n)
        else if (opcode[0] == 0b11100000) {
            uint8_t data = opcode[1];
            uint8_t addr = (0xFF << 8) | data;

            bus->write(addr, registers.A);
            return { , 2 };
        }

        print_error("Invalid opcode passed to LDH");
    }

    parse_instr_info NOP(uint8_t opcode[]) {
        return { , 1 };
    }

    parse_instr_info OR(uint8_t opcode[]) {
        // OR (HL): Bitwise OR (indirect HL)
        if (opcode[0] == 0b10110110) {
            uint8_t or_value = fetch(registers.HL);

            registers.A |= or_value;

            registers.flags.Z = registers.A == 0;
            registers.flags.N = 0;
            registers.flags.H = 0;
            registers.flags.C = 0;
            return { , 1 };
        }
        // OR r: Bitwise OR (register)
        else if ((opcode[0] >> 3) == 0b10110) {
            uint8_t or_value = register_8_index_read(opcode[0] & 0b0111);

            registers.A |= or_value;

            registers.flags.Z = registers.A == 0;
            registers.flags.N = 0;
            registers.flags.H = 0;
            registers.flags.C = 0;
            return { , 1 };
        }
        // OR n: Bitwise OR (immediate)
        else if (opcode[0] == 0b11110110) {
            uint8_t or_value = opcode[1];

            registers.A |= or_value;

            registers.flags.Z = registers.A == 0;
            registers.flags.N = 0;
            registers.flags.H = 0;
            registers.flags.C = 0;
            return { , 2 };
        }
        print_error("Invalid opcode passed to OR");
    }

    parse_instr_info POP(uint8_t opcode[]) {
        uint8_t rr = ((opcode[0] & 0b00110000) >> 4);

        uint8_t lsb = pop_stack(uint8_t opcode[]);
        uint8_t msb = pop_stack(uint8_t opcode[]);
        uint16_t data = (msb << 8) | lsb;

        register_16_index_write(rr, data);

        return { M_TO_T_CYC(3), 1 , false };
    }

    parse_instr_info PREF(uint8_t opcode[]) {
        registers.PC++;
        opcode[0] = fetch(registers.PC);

        instr_ret_info ret_info = (this->*prefix_instructions[opcode].operation)(uint8_t opcode[]);

        // The two following lines can be removed.
        // They're here to keep our convention of not modifying PC inside the instruction
        // (Unless it's an instruction that explicitly modifies PC)
        registers.PC--;
        // ret_info.cycles += 1;

        return ret_info;
    }

    parse_instr_info PUSH(uint8_t opcode[]) {
        // PUSH rr: Push to stack
        uint8_t rr = ((opcode[0] & 0b00110000) >> 4);
        uint16_t data = register_16_index_read(rr);

        uint8_t msb = MSB(data);
        uint8_t lsb = LSB(data);
        push_stack(msb);
        push_stack(lsb);

        return { , 1 };
    }

    parse_instr_info RET(uint8_t opcode[]) {
        // RET cc: Return from function (conditional)
        if (opcode[0] >> 5 == 0b110) {
            bool c_m = opcode[0] & 0b00010000;
            bool c_l = opcode[0] & 0b00001000;

            uint8_t lsb = pop_stack(uint8_t opcode[]);
            uint8_t msb = pop_stack(uint8_t opcode[]);
            uint16_t nn = (msb << 8) | lsb;

            bool condition = CC(c_l, c_m);
            if (condition) {
                registers.PC = nn;
                return { , 1 };
            }
            return { , 1 };
        }
        // RET: Return from function
        else if (opcode[0] == 0b11001001) {
            uint8_t lsb = pop_stack(uint8_t opcode[]);
            uint8_t msb = pop_stack(uint8_t opcode[]);
            uint16_t nn = (msb << 8) | lsb;

            registers.PC = nn;

            return { , 1 };
        }
        print_error("Invalid opcode passed to RET");
    }

    parse_instr_info RETI(uint8_t opcode[]) {
        // RETI: Return from interrupt handler
        uint8_t lsb = pop_stack(uint8_t opcode[]);
        uint8_t msb = pop_stack(uint8_t opcode[]);
        uint16_t nn = (msb << 8) | lsb;

        registers.PC = nn;
        registers.IME = true;

        return { , 1 };
    }

    parse_instr_info RLA(uint8_t opcode[]) {
        // RLA: Rotate left accumulator (0b00010111)
        uint8_t prev_a_value = registers.A;
        uint8_t result = (registers.A << 1) | registers.flags.C;
        registers.A = result;
        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = (prev_a_value & 0b10000000) != 0;
        return { , 1 };
    }

    parse_instr_info RLCA(uint8_t opcode[]) {
        // RLCA: Rotate left accumulator (0b00000111)
        uint8_t prev_a_value = registers.A;
        uint8_t result = (registers.A << 1) | (registers.A >> 7);
        registers.A = result;
        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = (prev_a_value & 0b10000000) != 0;
        return { , 1 };
    }

    parse_instr_info RRA(uint8_t opcode[]) {
        // RRA: Rotate right accumulator (0b00011111)
        uint8_t prev_a_value = registers.A;
        uint8_t result = (registers.A >> 1) | (registers.flags.C << 7);
        registers.A = result;
        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = (prev_a_value & 0b00000001) != 0;
        return { , 1 };
    }

    parse_instr_info RRCA(uint8_t opcode[]) {
        // RRCA: Rotate right accumulator (0b00001111)
        uint8_t prev_a_value = registers.A;
        uint8_t result = (registers.A >> 1) | (registers.A << 7);
        registers.A = result;
        registers.flags.Z = 0;
        registers.flags.N = 0;
        registers.flags.H = 0;
        registers.flags.C = (prev_a_value & 0b00000001) != 0;
        return { , 1 };
    }

    parse_instr_info RST(uint8_t opcode[]) {
        // RST n: Restart / Call function (implied) (opcode[0] 0b11xxx111)
        uint8_t lsb = opcode[0] & 0b00111000;
        uint8_t msb = 0x00;
        uint16_t nn = (msb << 8) | lsb;

        push_stack(MSB(registers.PC));
        push_stack(LSB(registers.PC));

        registers.PC = nn;

        return { , 1 };
    }

    parse_instr_info SBC(uint8_t opcode[]) {
        //SBC n : Subtract with carry(immediate)
        if (opcode[0] == 0b11011110) {
            uint8_t sub_value = opcode[1];

            return{ "SBC " + std::to_string(sub_value), 2 };
        }
        //SBC(HL) : Subtract with carry(indirect HL)
        else if (opcode[0] == 0b10011110) {
            return { "SBC [HL]", 1 };
        }
        // SBC r: Subtract with carry (register)
        else if ((opcode[0] >> 3) == 0b10011) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return  { "SBC " + reg_name, 1 };
        }
        print_error("Invalid opcode passed to SBC");
    }

    parse_instr_info SCF(uint8_t opcode[]) {
        // SCF: Set carry flag (opcode[0] 0b00110111)
        return{ "SCF", 1 };
    }

    parse_instr_info STOP(uint8_t opcode[]) {
        print_error("IT'S TIME TO STOP! The programmer has a nap. Hold out! Programmer!");
    }

    parse_instr_info SUB(uint8_t opcode[]) {
        //SUB(HL) : Subtract(indirect HL)
        if (opcode[0] == 0b10010110) {
            return { "SUB [HL]", 1 };
        }
        // SUB n: Subtract (immediate)
        else if (opcode[0] == 0b11010110) {
            uint8_t sub_value = opcode[1];

            return{ "SUB " + std::to_string(sub_value), 2 };
        }
        // SUB r: Subtract (register)
        else if ((opcode[0] >> 3) == 0b10010) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "SUB " + reg_name, 1 };
        }

        print_error("Invalid opcode passed to SUB");
    }

    parse_instr_info XOR(uint8_t opcode[]) {
        // XOR(HL) : Bitwise XOR(indirect HL)
        if (opcode[0] == 0b10101110) {
            return { "XOR [HL]", 1 };
        }
        // XOR n: Bitwise XOR (immediate)
        else if (opcode[0] == 0b11101110) {
            uint8_t xor_value = opcode[1];

            return { "XOR " + std::to_string(xor_value), 2 };
        }
        // XOR r: Bitwise XOR (register)
        else if ((opcode[0] >> 3) == 0b10101) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "XOR " + reg_name, 1 };
        }
        print_error("Invalid opcode passed to XOR");
    }

    parse_instr_info XXX(uint8_t opcode[]) {
        print_error("Invalid opcode: Out of range");
    }

    // Prefixed instructions
    parse_instr_info BIT(uint8_t opcode[]) {
        // BIT b, (HL): Test bit (indirect HL) (0b01xxx110)
        if ((opcode[0] & 0b11000111) == 0b01000110) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;

            return { "BIT " + std::to_string(bit_index), 2 };
        }

        // BIT b, r: Test bit (register) (0b01bbbrrr)
        else if ((opcode[0] & 0b11000000) == 0b01000000) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "BIT " + std::to_string(bit_index) + reg_name, 2 };
        }

        print_error("Invalid opcode passed to BIT");
    }

    parse_instr_info RES(uint8_t opcode[]) {
        // RES b, r: Reset bit (register) (0b10bbbrrr)
        if ((opcode[0] & 0b11000000) == 0b10000000) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "RES " + std::to_string(bit_index) + reg_name, 2 };
        }

        // RES b, (HL): Reset bit (indirect HL) (0b10bbb110)
        else if ((opcode[0] & 0b11000111) == 0b10000110) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;
            return { "RES " + std::to_string(bit_index), 2 };
        }


        print_error("Invalid opcode passed to RES");
    }

    parse_instr_info RL(uint8_t opcode[]) {
        // RL r: Rotate left (register) (0b00010xxx)
        if (opcode[0] >> 3 == 0b00010) {
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "RL " + reg_name , 2 };
        }

        // RL (HL): Rotate left (indirect HL)
        else if (opcode[0] == 0b00010110) {
            return { "RL [HL]", 2 };
        }

        print_error("Invalid opcode passed to RL");
    }

    parse_instr_info RLC(uint8_t opcode[]) {
        // RLC [HL]: Rotate left indirect HL (0b00000110)
        if (opcode[0] == 0b00000110) {
            return { "RLC [HL]", 2 };
        }

        // RLC r: Rotate left (register) (0b00000xxx)
        else if ((opcode[0] >> 3) == 0b00000) {
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "RLC " + reg_name, 2 };
        }

        print_error("Invalid opcode passed to RLC");
    }

    parse_instr_info RR(uint8_t opcode[]) {
        // RR r: Rotate right (register) (0b00011xxx)
        if (opcode[0] >> 3 == 0b00011) {
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);
            return { "RR " + reg_name, 2 };
        }

        // RR (HL): Rotate right (indirect HL)
        else if (opcode[0] == 0b00011110) {
            return { "RR [HL]", 2 };
        }

        print_error("Invalid opcode passed to RR");
    }

    parse_instr_info RRC(uint8_t opcode[]) {
        // RRC (HL): Rotate right circular (indirect HL) 00001110
        if (opcode[0] == 0b00001110) {
            return { "RRC [HL]", 2 };
        }
        // RRC r: Rotate right circular (register) 0b00001xxx
        else if (opcode[0] >> 3 == 0b00001) {
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "RRC " + reg_name , 2 };
        }

        print_error("Invalid opcode passed to RRC");
    }

    parse_instr_info SET(uint8_t opcode[]) {
        // SET b, (HL): Set bit (indirect HL) (0b11bbb110)
        if ((opcode[0] & 0b11000111) == 0b11000110) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;
            return { "SET " + std::to_string(bit_index) , 2 };
        }

        // SET b, r: Set bit (register) (0b11bbbrrr)
        else if ((opcode[0] & 0b11000000) == 0b11000000) {
            uint8_t bit_index = (opcode[0] & 0b00111000) >> 3;
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "SET " + std::to_string(bit_index) + reg_name , 2 };
        }
        print_error("Invalid opcode passed to SET");
    }

    parse_instr_info SLA(uint8_t opcode[]) {
        // Shift Left Arithmetically the byte pointed to by HL.
        if (opcode[0] == 0b00101110) {
            return { "SLA [HL]", 2 };
        }
        // Shift Left Arithmetically register r8.
        else if ((opcode[0] >> 3) == 0b00101) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "SLA " + reg_name, 2 };
        }

        print_error("Invalid opcode passed to SLA");
    }

    parse_instr_info SRA(uint8_t opcode[]) {
        // Shift Left Arithmetically the byte pointed to by HL.
        if (opcode[0] == 0b00111110) {
            return { "SLA [HL]", 2 };
        }
        // Shift Right Arithmetically register r8(bit 7 of r8 is unchanged).
        else if ((opcode[0] >> 3) == 0b00111) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "SRA " + reg_name, 2 };
        }
        print_error("Invalid opcode passed to SRA");
    }
    parse_instr_info SRL(uint8_t opcode[]) {
        // Shift Right Logically the byte pointed to by HL.
        if (opcode[0] == 0b00111110) {
            return { "SLR [HL]", 2 };
        }
        // Shift Right Logically register r8.
        else if ((opcode[0] >> 3) == 0b00111) {
            uint8_t reg_index = opcode[0] & 0b0111;
            std::string reg_name = register_8_to_register_name(reg_index);

            return { "SLR " + reg_name, 2 };
        }
        print_error("Invalid opcode passed to SRL");
    }

    parse_instr_info SWAP(uint8_t opcode[]) {
        // SWAP (HL): Swap nibbles (indirect HL)
        if (opcode[0] == 0b00110110) {
            return { "SWAP [HL]", 2 };
        }

        // SWAP r: Swap nibbles (register)
        else if ((opcode[0] >> 3) == 0b00110) {
            uint8_t reg_index = opcode[0] & 0b00000111;
            std::string reg_name = register_8_to_register_name(reg_index);
            return { "SWAP" + reg_name, 2 };
        }

        print_error("Invalid opcode passed to SWAP");
    }
}