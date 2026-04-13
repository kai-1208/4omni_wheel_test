#ifndef WT901_HPP
#define WT901_HPP

#include "mbed.h"

class WT901 {
public:
    WT901(PinName tx, PinName rx, int baudrate = 115200);

    // データ更新
    void update();

    // === 個別に値を取得する関数（戻り値で直接受け取れます） ===
    float getRoll();
    float getPitch();
    float getYaw();

    float getAccelX();
    float getAccelY();
    float getAccelZ();

    float getGyroX();
    float getGyroY();
    float getGyroZ();

private:
    BufferedSerial _imu; 
    
    uint8_t _rx_buffer[11];
    int _state;

    float _accel[3];
    float _gyro[3];
    float _angle[3];

    void parse_packet();
};

#endif // WT901_HPP