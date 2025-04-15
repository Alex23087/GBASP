#pragma once
#include <string>

#ifdef DEBUG
#define ASSERT(x) if(x) {print_error("Assertion failed: x");}
#else
#define ASSERT(x)
#endif

#define MSB(x) (x & 0xF0)
#define LSB(x) (x & 0x0F)

#define CARRY_4(x, y) ((x & 0b00001111) > (y & 0b00001111))
#define CARRY_8(x, y) (x > y)


void print_error(std::string error);