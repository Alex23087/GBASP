/**
 * Creates a dummy ROM file that call
 */

#include <cstdint>
void create_rom(uint8_t buffer[]) {
    buffer[0] = 0b10001110; // ADC (HL): Add with carry (indirect HL)
    // TODO add the rest of the instructions
}