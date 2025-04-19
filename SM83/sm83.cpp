/**
/ This file contains the main cycle of the SM83 processor
*/
#include "sm83.hpp"
#include "utils.hpp"
#include <cstdint>

SM83::SM83() {
    reset();

    // TODO Initialise both instructions and prefix_instructions
}

SM83::~SM83() {
    // Destructor
}

void SM83::run() {
    while (cycles > 0) {
        // Fetch

        // Decode & Execute


        // Update PC

    }
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
    cycles = 0;
}

void SM83::connect_to_bus(Bus* bus) {
    this->bus = bus;
}


void SM83::decode_execute(uint8_t opcode) {
    // WARNING: remember that some ops change PC (e.g., CALL)
    // WARNING: Remember to handle IME_DEFER
    // WARNING: Remember to handle interrupts
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
            print_error("Invalid index passed register_8_index_read");
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
        case 1:
            return registers.DE;
        case 2:
            return registers.HL;
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
    // TODO: Add proper bus read/write function calls
    //return bus.read(addr);
}

void SM83::push_stack(uint8_t data) {
    registers.SP -= 1;
    // TODO: Add proper bus read/write function calls
    // bus.write(registers.SP, data);
}

uint8_t SM83::pop_stack() {
    registers.SP += 1;
    // TODO: Add proper bus read/write function calls
    // return bus.read(registers.SP);
}