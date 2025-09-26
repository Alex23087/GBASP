#include <iostream>
#include "Hardware/gameboy.hpp"
#include "UI/main_ui.hpp"
#include "tests/sm83_test.hpp"

int main() {
    std::cout << "I'M ALIVE!!!!!!!!" << std::endl;

    // Initialize the GameBoy hardware
    GameBoy gameboy = initialize_gameboy();

    sm83_test(gameboy);

    // Initialize UI
    main_d(0, NULL);

    return 0;
}