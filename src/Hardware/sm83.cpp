/**
/ This file contains the main cycle of the SM83 processor
*/
#include "sm83.hpp"
#include "utils.hpp"
#include <cstdint>

SM83::SM83() {
    printf("INITIALIZING SM83\n");
    fill_instruction_array();
    reset();
}

SM83::~SM83() {
    // Destructor
}

/** This function prints all the registers content.
 */
void SM83::print_registers() {
    printf("PC = %u\t", this->registers.PC);
    printf("IR = %u\t", this->registers.IR);
    printf("IE = %u\t", this->registers.IE);
    printf("A = %u\n", this->registers.A);

    printf("F = %u\t", this->registers.F);
    printf("B = %u\t", this->registers.B);
    printf("C = %u\t", this->registers.C);
    printf("BC = %u\t", this->registers.BC);

    printf("D = %u\t", this->registers.D);
    printf("E = %u\t", this->registers.E);
    printf("DE = %u\n", this->registers.DE);

    printf("H = %u\t", this->registers.H);
    printf("L = %u\t", this->registers.L);
    printf("HL = %u\t", this->registers.HL);

    printf("SP = %u\n", this->registers.SP);

    printf("F = %u\t", this->registers.F);
    printf("IF = %u\t", this->registers.IF);

    printf("IME = %u\t", this->registers.IME);
    printf("IME_DEFER = %u\n", this->registers.IME_DEFER);

    printf("\n");
}

/** This function handles the execution of the CPU for a specified number of T-cycles.
 * The function returns the number of cycles exceeded for handling the edge case where the
 * last completed instruction required more cycles than the ones available.
 */
uint8_t SM83::run(uint32_t cycles) {
    uint8_t exceeding_cycles = 0;
#if GBASP_DEBUG
    print_registers();
#endif
    while (cycles > 0) {
        // Fetch
        registers.IR = fetch(registers.PC);
#if GBASP_DEBUG
        printf("Fetched opcode: %02X at PC: %04X\n", registers.IR, registers.PC);
#endif

        // Decode & Execute
        instr_ret_info ret_info = (this->*(instructions[registers.IR].operation))();

        // Update PC
        if (!ret_info.has_set_PC) {
            registers.PC += ret_info.pc_incr;
        }

        // Update cycles
        if (ret_info.cycles > cycles) {
            exceeding_cycles = ret_info.cycles - cycles;
            cycles = 0;
        } else {
            cycles -= ret_info.cycles;
        }
        this->cycle_count += ret_info.cycles;
#if GBASP_DEBUG
        printf("Cycle count: %u\n", this->cycle_count);
        print_registers();
#endif
    }

    return exceeding_cycles;
}

void SM83::reset() {
    registers.IR = 0;
    registers.IE = 0;
    registers.A = 0;
    registers.F = 0;
    registers.BC = 0;
    registers.DE = 0;
    registers.HL = 0;
    registers.PC = 0;
    registers.SP = 0;
    registers.IME = 0;
    registers.IME_DEFER = 0;
    cycle_count = 0;
}

void SM83::connect_to_bus(Bus* bus) {
    this->bus = bus;
}

uint8_t SM83::register_8_index_read(uint8_t index) {
    switch (index) {
        case 0:
            return registers.B;
        case 1:
            return registers.C;
        case 2:
            return registers.D;
        case 3:
            return registers.E;
        case 4:
            return registers.H;
        case 5:
            return registers.L;
        case 7:
            return registers.A;
        default:
            print_error("Invalid index passed to register_8_index_read");
    }
}

void SM83::register_8_index_write(uint8_t index, uint8_t data) {
    switch (index) {
        case 0:
            registers.B = data;
            return;
        case 1:
            registers.C = data;
            return;
        case 2:
            registers.D = data;
            return;
        case 3:
            registers.E = data;
            return;
        case 4:
            registers.H = data;
            return;
        case 5:
            registers.L = data;
            return;
        case 7:
            registers.A = data;
            return;
        default:
            print_error("Invalid index passed register_8_index_write");
    }
}

uint16_t SM83::register_16_index_read(uint8_t index) {
    switch (index) {
        case 0:
            return registers.BC;
        case 1:// TODO: Add proper bus read/write function calls
            // 
        case 3:
            return registers.SP;
        default:
            print_error("Invalid index passed register_16_index_read");
    }
}

void SM83::register_16_index_write(uint8_t index, uint16_t data) {
    switch (index) {
        case 0:
            registers.BC = data;
            return;
        case 1:
            registers.DE = data;
            return;
        case 2:
            registers.HL = data;
            return;
        case 3:
            registers.SP = data;
            return;
        default:
            print_error("Invalid index passed register_16_index_write");
    }
}

uint8_t SM83::fetch(uint16_t addr) {
    return bus->read(addr);
}

void SM83::push_stack(uint8_t data) {
    registers.SP -= 1;
    bus->write(registers.SP, data);
}

uint8_t SM83::pop_stack() {
    registers.SP += 1;
    return bus->read(registers.SP);
}