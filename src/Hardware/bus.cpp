#include "bus.hpp"
#include "utils.hpp"

Bus::Bus() {
    printf("INITIALIZING BUS\n");
}

Bus::~Bus() {
}


void Bus::write(uint16_t address, uint8_t data) {
    DeviceMapping* device = find_device(address);
    uint16_t offset = address - device->first_address;
    device->device->write(offset, data);
}

uint8_t Bus::read(uint16_t address) {
    DeviceMapping* device = find_device(address);
    uint16_t offset = address - device->first_address;
    device->device->read(offset);
}

void Bus::attach_device(uint16_t first_address, uint16_t last_address, Device* device) {
    devices.push_back({ first_address, last_address, device });
    std::sort(devices.begin(), devices.end(), [](const DeviceMapping& a, const DeviceMapping& b) {
        return (a.last_address - a.first_address) > (b.last_address - b.first_address);
        }
    );
}

Bus::DeviceMapping* Bus::find_device(uint16_t address) {
    for (DeviceMapping& device : devices) {
        if (address >= device.first_address && address <= device.last_address) {
            return &device;
        }
    }
    print_error("Device not found for address: " + std::to_string(address));
}
