#include <iostream>
#include "SM83/sm83.hpp"
#include "SM83/bus.hpp"
#include "SM83/ram.hpp"
#include "UI/debugger/debugger.hpp"

int main() {
    std::cout << "Hello, Perkyello e Sal I'm alive!" << std::endl;

    SM83 sm83;
    Bus bus;
    Ram ram;
    sm83.connect_to_bus(&bus);
    bus.attach_device(0x0000, 0xFFFF, &ram);
    sm83.run();


    main_d(0, NULL);

    return 0;
}