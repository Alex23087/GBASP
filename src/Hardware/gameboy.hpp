#pragma once

#include "sm83.hpp"
#include "bus.hpp"
#include "ram.hpp"

typedef struct GameBoy {
    SM83 sm83;
    Bus bus;
    RAM ram;
} GameBoy;

GameBoy& initialize_gameboy();