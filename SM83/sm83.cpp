/**
/ This file contains the main cycle of the SM83 processor
*/
#include <cstdint>
#include "sm83.hpp"

SM83::SM83() {
    reset();
}

SM83::~SM83() {
    // Destructor
}

void SM83::run() {
    while (cycles > 0) {
        // Fetch

        // Decode

        // Execute

        // Update PC

    }
}

void SM83::reset() {
    registers.IR = 0;
    registers.IE = 0;
    registers.AF = 0;
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