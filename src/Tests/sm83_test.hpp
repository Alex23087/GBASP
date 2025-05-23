/**
 * For each instruction we test it by executing it and checking that the state
 * of the CPU and the dummy memory is correct. The test is handled by the dummy
 * memory which will also contain the program ROM referring to the test.
*/
#pragma once

void sm83_test();