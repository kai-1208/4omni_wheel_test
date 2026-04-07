#ifndef BNO055_UART_HPP
#define BNO055_UART_HPP
#include "mbed.h"
class BNO055Uart {
public:
    enum class Response : uint8_t { SUCCESS = 0x01, NO_RESPONSE = 0xFF };
    struct EulerAngles { float roll; float pitch; float yaw; };
    BNO055Uart(PinName tx, PinName rx);
    bool begin(chrono::milliseconds timeout = 1000ms);
    bool update();
    EulerAngles getEuler() const;
private:
    enum Register : uint8_t { EUL_DATA_X_LSB = 0x1A, OPR_MODE = 0x3D };
    bool reg_write(uint8_t addr, uint8_t data);
    bool reg_read(uint8_t addr, uint8_t* buffer, uint8_t len);
    UnbufferedSerial _serial;
    EulerAngles _euler;
};
#endif