#include "serial_read.hpp"
#include "mbed.h"
#include "key.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>

std::vector<double> to_numbers(const std::string &input) {
    std::vector<double> numbers;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, ':')) { // ':'で区切る
        if (!token.empty() && token.back() == '|') {  // 最後の '|' を削除
            token.pop_back();
        }
        numbers.push_back(std::stod(token)); // 文字列をdoubleに変換
    }
    return numbers;
}

std::string serial_unit::read_serial()
{
    static std::string str = "";
    char buf[1];

    while (men_serial.readable())
    {
        men_serial.read(buf, 1);
        if (buf[0] == '|')
        {
            std::string buff_str = str;
            str.clear();
            return buff_str;
        }
        else
        {
            str += buf[0];
        }
    }
    return "";
}


void serial_read() {
    int serial_count = 0;
    mbed::HighResClock::time_point now_time = HighResClock::now();
    mbed::HighResClock::time_point pre_time = HighResClock::now();
    while (1) {
        now_time = HighResClock::now();
        // printf("serial_read\n");
        if (now_time - pre_time > 1ms) {
            std::string msg = serial.read_serial();
            if (msg != "") {
                is_serial_timeout = false;
                serial_count = 0;
                if (msg[0] == 'n') {
                    move_aa(msg);
                } else {
                    key_puress(msg);
                }
            } else {
                serial_count++;
            }
            
            if (serial_count > 1000) { // 1秒間メッセージが来ない場合
                is_serial_timeout = true;
            }
        }
        // ThisThread::sleep_for(1ms);
    }
}