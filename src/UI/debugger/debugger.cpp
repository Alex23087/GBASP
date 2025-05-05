/**
 * This file handes the debugger of the emulator.

 */

#include "debugger.hpp"
#include "imgui.h"
#include "imgui_memory_editor.h"

void show_debugger() {
    static MemoryEditor mem_edit_1;
    static char data[0x10000];
    size_t data_size = 0x10000;
    mem_edit_1.DrawWindow("Memory Editor", data, data_size);
}