#include <iostream>
#include "sm83.hpp"
#include "bus.hpp"
#include "ram.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    SM83 sm83;
    Bus bus;
    Ram ram;
    bus.attach_device(0x0000, 0xFFFF, &ram);
    sm83.run();


    return 0;
}