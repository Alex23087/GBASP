#include <iostream>
#include "Hardware/gameboy.hpp"
#include "UI/main_ui.hpp"

int main() {
    std::cout << "I'M ALIVE!!!!!!!!" << std::endl;

    // Initialize the GameBoy hardware
    GameBoy gameboy = initialize_gameboy();

    // Initialize UI
    main_d(0, NULL);

    return 0;
}