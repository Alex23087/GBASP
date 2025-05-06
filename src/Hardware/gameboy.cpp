/**
 * This file handles the initialization of the emulated GameBoy hardware.
 */

#include "sm83.hpp"
#include "bus.hpp"
#include "ram.hpp"
#include "gameboy.hpp"

GameBoy& initialize_gameboy() {
    GameBoy gameboy;
    gameboy.sm83.connect_to_bus(&gameboy.bus);
    gameboy.bus.attach_device(0x0000, 0xFFFF, &gameboy.ram);
    return gameboy;
}