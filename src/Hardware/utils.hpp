#pragma once
#include <string>

#ifdef DEBUG
#define ASSERT(x) if(x) {print_error("Assertion failed: x");}
#else
#define ASSERT(x)
#endif

#define MSB(x) ((x & 0xFF00) >> 4)
#define LSB(x) (x & 0x00FF)

#define CARRY_4(x, y) ((x & 0b00001111) > (y & 0b00001111))
#define CARRY_8(x, y) (x > y)

// #define TO_HEX(n) (std::to_string(n))
// #define TO_HEX(n) (std::format("{:x}", n))
std::string TO_HEX(uint8_t n);

[[noreturn]] void print_error(std::string error);