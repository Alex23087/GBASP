#include <cstdint>
#include <array>
using namespace std;

class Ram : public Device {
public:
    Ram();
    ~Ram();
    void write(uint16_t address, uint8_t data);
    uint8_t read(uint16_t address);

private:
    std::array<uint8_t, 64 * 1024> memory = { 0 };
};