/**
 * For each instruction we test it by executing it and checking that the state
 * of the CPU and the dummy memory is correct. The test is handled by the dummy
 * memory which will also contain the program ROM referring to the test.
*/

#include "sm83_test.hpp"
#include "test_rom.hpp"

#include "../Hardware/initialize_parser_arrays.hpp"
#include "../Hardware/instructions_parser.hpp"

void sm83_test(GameBoy& gameboy) {

    // Load the manual test commands into memory
    uint8_t test_rom[512] = { 0 };
    create_rom(test_rom);
    for (size_t i = 0; i < sizeof(test_rom); ++i) {
        gameboy.ram.write(i, test_rom[i]);
    }

    // Execute the test instructions
    gameboy.sm83.run(32);

    Parser::fill_instruction_array();
    Parser::parse_instr_info pii;
    uint32_t offset = 0;
    for (int i = 0; i < 3; i++) {
        pii = Parser::parse_instruction(test_rom + offset);
        printf("Parsed instruction: %s\n", pii.operation_pp.c_str());
        offset += pii.pc_incr;
    }

    for (uint8_t i = 0; i < 255; i++) {
        uint8_t opcode[4] = { 0 };
        opcode[0] = i;
        printf("%s\n", Parser::parse_instruction(opcode).operation_pp.c_str());
    }
}
