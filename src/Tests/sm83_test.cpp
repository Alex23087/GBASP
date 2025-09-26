/**
 * For each instruction we test it by executing it and checking that the state
 * of the CPU and the dummy memory is correct. The test is handled by the dummy
 * memory which will also contain the program ROM referring to the test.
*/

#include "sm83_test.hpp"
#include "test_rom.hpp"

void sm83_test(GameBoy& gameboy) {

    // Load the manual test commands into memory
    uint8_t test_rom[512] = { 0 };
    create_rom(test_rom);
    for (size_t i = 0; i < sizeof(test_rom); ++i) {
        gameboy.ram.write(i, test_rom[i]);
    }

    // Execute the test instructions
    gameboy.sm83.run(100);
}
