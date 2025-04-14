#include "utils.hpp"
#include <iostream>
#include <string>

void print_error(std::string error) {
    std::cerr << "Error: " << error << std::endl;
    exit(-1);
}