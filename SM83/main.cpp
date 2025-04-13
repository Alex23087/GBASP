#include <iostream>
#include "sm83.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    SM83 sm83;
    sm83.reset();
    sm83.run();


    return 0;
}