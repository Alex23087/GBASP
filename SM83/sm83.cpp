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
    cycles = 0;
}


void SM83::write(uint16_t address, uint8_t data) {

}

uint8_t SM83::read(uint16_t address) {

}

void SM83::writeWord(uint16_t address, uint16_t data) {

}

uint16_t SM83::readWord(uint16_t address) {

}

void SM83::decode_execute(uint8_t opcode) {
    // 
}

uint8_t SM83::register_from_index(uint8_t index) {
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
            print_error("Invalid index passed register_from_index");
    }
}

uint8_t SM83::fetch(uint16_t addr) {
    //return bus.read(addr);
}