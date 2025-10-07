/**
 * Creates a dummy ROM file that call
 */

#include <cstdint>
void create_rom(uint8_t buffer[]) {
    buffer[0] = 0b00111110; // LD IMMEDIATE 0b00rrr110 (A = 001)
    buffer[1] = 0000000001; // IMMEDIATE (1)
    buffer[2] = 0b00000110; // LD IMMEDIATE 0b00rrr110 (B = 000)
    buffer[3] = 0b00000010; // IMMEDIATE (2)
    buffer[4] = 0xDC; // ADD A, B: Add register to A (A = A + B)
    // buffer[2] = 0b10001110; // ADC (HL): Add with carry (indirect HL)
    // TODO add the rest of the instructions
}