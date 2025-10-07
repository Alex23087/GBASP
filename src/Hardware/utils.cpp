#include "utils.hpp"
#include <iostream>
#include <string>

[[noreturn]] void print_error(std::string error) {
    std::cerr << "Error: " << error << std::endl;
    exit(-1);
}

std::string TO_HEX(uint8_t n) {
    char buffer[5]; // "0x" + 2 hex digits + null terminator
    snprintf(buffer, sizeof(buffer), "0x%02X", n);
    return std::string(buffer);
}