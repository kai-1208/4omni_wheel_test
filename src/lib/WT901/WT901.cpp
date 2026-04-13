#include "WT901.hpp"

WT901::WT901(PinName tx, PinName rx, int baudrate)
    : _imu(tx, rx, baudrate), _state(0) {
    for (int i = 0; i < 3; i++) {
        _accel[i] = 0.0f;
        _gyro[i]  = 0.0f;
        _angle[i] = 0.0f;
    }
}

void WT901::update() {
    uint8_t c;
    while (_imu.readable()) {
        _imu.read(&c, 1);
        
        if (_state == 0 && c == 0x55) {
            _rx_buffer[0] = c;
            _state = 1;
        } else if (_state == 1) {
            if (c >= 0x51 && c <= 0x5A) {
                _rx_buffer[1] = c;
                _state = 2;
            } else {
                _state = 0;
            }
        } else if (_state >= 2 && _state < 11) {
            _rx_buffer[_state] = c;
            _state++;
            
            if (_state == 11) {
                parse_packet();
                _state = 0;
            }
        }
    }
}

void WT901::parse_packet() {
    uint8_t checksum = 0;
    for(int i = 0; i < 10; i++) {
        checksum += _rx_buffer[i];
    }
    if(checksum != _rx_buffer[10]) return;

    switch(_rx_buffer[1]) {
        case 0x51: // 加速度
            _accel[0] = ((short)((_rx_buffer[3] << 8) | _rx_buffer[2])) / 32768.0f * 16.0f;
            _accel[1] = ((short)((_rx_buffer[5] << 8) | _rx_buffer[4])) / 32768.0f * 16.0f;
            _accel[2] = ((short)((_rx_buffer[7] << 8) | _rx_buffer[6])) / 32768.0f * 16.0f;
            break;
            
        case 0x52: // 角速度
            _gyro[0] = ((short)((_rx_buffer[3] << 8) | _rx_buffer[2])) / 32768.0f * 2000.0f;
            _gyro[1] = ((short)((_rx_buffer[5] << 8) | _rx_buffer[4])) / 32768.0f * 2000.0f;
            _gyro[2] = ((short)((_rx_buffer[7] << 8) | _rx_buffer[6])) / 32768.0f * 2000.0f;
            break;
            
        case 0x53: // 角度
            _angle[0] = ((short)((_rx_buffer[3] << 8) | _rx_buffer[2])) / 32768.0f * 180.0f;
            _angle[1] = ((short)((_rx_buffer[5] << 8) | _rx_buffer[4])) / 32768.0f * 180.0f;
            _angle[2] = ((short)((_rx_buffer[7] << 8) | _rx_buffer[6])) / 32768.0f * 180.0f;
            break;
    }
}

// === ゲッター関数（個別の値を返す） ===
float WT901::getRoll()  { return _angle[0]; }
float WT901::getPitch() { return _angle[1]; }
float WT901::getYaw()   { return _angle[2]; }

float WT901::getAccelX(){ return _accel[0]; }
float WT901::getAccelY(){ return _accel[1]; }
float WT901::getAccelZ(){ return _accel[2]; }

float WT901::getGyroX() { return _gyro[0];  }
float WT901::getGyroY() { return _gyro[1];  }
float WT901::getGyroZ() { return _gyro[2];  }