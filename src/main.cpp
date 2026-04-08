#include "cmath"
#include "key.hpp"
#include "mbed.h"
#include "serial_read.hpp"
#include "BNO055Uart.hpp"
#include "pid.hpp"
#include "c610.hpp"

#define M_PI 3.14159265358979323846
#define SPEED_SCALE 8000

BufferedSerial pc(USBTX, USBRX, 115200);
serial_unit serial(pc);

// imu設定
BNO055Uart imu(PA_9, PA_10);
bool imu_available = false;
float yaw_offset = 0.0f;

// can設定
CAN can1(PA_11, PA_12, (int)1e6);
CAN can2(PB_12, PB_13, (int)1e6);
C610 DJI1(can1);

// pid設定
PID wheel_pid[4] = {
    PID(0.6, 0.05, 0.0, PID::Mode::VELOCITY),
    PID(0.6, 0.05, 0.0, PID::Mode::VELOCITY),
    PID(0.6, 0.05, 0.0, PID::Mode::VELOCITY),
    PID(0.6, 0.05, 0.0, PID::Mode::VELOCITY)
};

double wheel_pid_output[4] = {0};

// aifubdsih
uint8_t data1[8] = {};
double wheel_speeds[4];

double theta = 0.0;
double speed = 0.0;
double relative_yaw = 0.0;

int16_t pwm[4] = {0};

/**
 * @brief imuを更新
 */
void imu_polling() {
    int consecutive_fail = 0;
    const int FAIL_THRESHOLD = 8; // 8回連続で失敗したら再初期化

    while (true) {
        if (imu_available) {
            if (imu.update()) {
                consecutive_fail = 0;
                // printf("imu: update success, yaw: %f\n", relative_yaw);
            } else {
                consecutive_fail++;
                // printf("imu: update failed: %d, yaw: %f\n", consecutive_fail, relative_yaw);
                // if (consecutive_fail == 1) {
                //     // 最初の失敗はログだけ
                //     printf("imu: read failed (1)\n");
                // }
            }

            if (consecutive_fail >= FAIL_THRESHOLD) {
                printf("imu: %d consecutive failures. Attempting re-init...\n", consecutive_fail);
                // 再初期化
                if (imu.begin()) {
                    printf("imu: re-init success\n");
                    imu.update();
                    // yaw_offset = imu.getEuler().yaw; // オフセットは再設定しない
                    consecutive_fail = 0;
                    imu_available = true;
                } else {
                    printf("imu: re-init failed\n");
                    wheel_speeds[4] = {0};
                    imu_available = false;
                    // 次は長めに待ってから再試行
                    ThisThread::sleep_for(500ms);
                    consecutive_fail = 0;
                }
            }
        } else {
            if (imu.begin()) {
                imu_available = true;
                printf("imu: became available\n");
            } else {
                ThisThread::sleep_for(500ms);
            }
        }
        ThisThread::sleep_for(20ms);
    }
}

/**
 * @brief pid制御 omniだけ
 */
void pid_control() {
    int wheel_min_driver_power = 100;
    int wheel_output_limits = 16000;
    for (int i = 0; i < 4; ++i) {
        wheel_pid[i].set_min_drive_power(wheel_min_driver_power);
        wheel_pid[i].set_output_limits(-wheel_output_limits, wheel_output_limits);
        wheel_pid[i].enable_anti_windup(true);
    }

    auto pre_time = HighResClock::now();
    while (true) {
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time - pre_time).count() / 1000000.0f;
        for (int i = 0; i < 4; ++i) {
            wheel_pid[i].set_dt(dt);
        }
        pre_time = now_time;

        ThisThread::sleep_for(10ms);
    }
}

/**
 * @brief imuからyaw角を取得
 */
void imu_get_yaw() {
    if (imu.begin()) {
        ThisThread::sleep_for(100ms); // imu安定するまで待つ
        imu.update();
        yaw_offset = imu.getEuler().yaw; // 最初のヨー角
        printf("Yaw offset set to: %.2f\n", yaw_offset);
    } else {
        printf("Failed to initialize BNO055.\n");
        while(1);
    }

    while (true) {
        if (imu.update()) {
            BNO055Uart::EulerAngles angles = imu.getEuler();
            relative_yaw = angles.yaw - yaw_offset; // ヨー角をoffset修正

            // 角度を-180～180度に
            if (relative_yaw > 180.0f) relative_yaw -= 360.0f;
            if (relative_yaw < -180.0f) relative_yaw += 360.0f;

            printf("Yaw: %7.2f\n", relative_yaw);
        } else {
            printf("Failed to update sensor data.\n");
        }
        ThisThread::sleep_for(100ms);
    }
}

/**
 * @brief ps4コントローラーの入力値の取得&4輪オムニ制御
 */
void move_aa(std::string msg) {
    msg.erase(0, 2);
    std::vector<double> joys_d = to_numbers(msg);
    std::vector<float> joys(joys_d.begin(), joys_d.end());
    for (auto &joy : joys) {
        if (joy > -0.08 && joy < 0.08) {
            joy = 0.0;
        }
    }
    float lx = joys[0];
    float ly = -joys[1]; // lスティックのy座標を反転
    float rx = joys[2];

    // IMUから現在のヨー角を取得
    if (imu_available) {
        BNO055Uart::EulerAngles angles = imu.getEuler();
        double current_yaw = angles.yaw - yaw_offset;

        // 角度を-180度から180度の範囲に正規化
        while (current_yaw > 180.0) current_yaw -= 360.0;
        while (current_yaw < -180.0) current_yaw += 360.0;
        relative_yaw = current_yaw;
    }

    // フィールド座標系を基準とした移動量の計算
    double yaw_rad = relative_yaw * M_PI / 180.0;
    theta = atan2(ly, lx);
    speed = hypot(lx, ly);

    // 各ホイールの速度を計算
    wheel_speeds[0] = (speed * cos(theta - M_PI/4 - yaw_rad) + rx) * SPEED_SCALE; // 右前
    wheel_speeds[1] = (speed * cos(theta - 3*M_PI/4 - yaw_rad) + rx) * SPEED_SCALE; // 右後
    wheel_speeds[2] = (speed * cos(theta - 5*M_PI/4 - yaw_rad) + rx) * SPEED_SCALE; // 左後
    wheel_speeds[3] = (speed * cos(theta - 7*M_PI/4 - yaw_rad) + rx) * SPEED_SCALE; // 左前


    for (int i = 0; i < 4; ++i) {
        wheel_pid_output[i] = wheel_pid[i].do_pid(DJI1.get_rpm(i+1));
        wheel_pid[i].set_goal(wheel_speeds[i]);
        DJI1.set_power(i+1, wheel_pid_output[i]);
        // if (i == 3) printf("%f\n", wheel_pid[i].do_pid(DJI1.get_rpm(i+1)));
        // else printf("%f, ", wheel_pid[i].do_pid(DJI1.get_rpm(i+1)));
        // if (i == 3) printf("%d\n", DJI1.get_rpm(i+1));
        // else printf("%d, ", DJI1.get_rpm(i+1));
    }

    // ↓はc610.hppを使用しないcan通信
    
    // int16_t pwm_outputs[4];
    // for (int i = 0; i < 4; ++i) {
    //     pwm_outputs[i] = static_cast<int16_t>(wheel_speeds[i]);
    // }

    // for (int i = 0; i < 4; ++i) {
    //     int16_t v = pwm_outputs[i];
    //     data1[i * 2]     = static_cast<uint8_t>((v >> 8) & 0xFF); // MSB
    //     data1[i * 2 + 1] = static_cast<uint8_t>(v & 0xFF);        // LSB
    // }

    // printf("%f, %f, %f, %f, %f, %f, %f\n", wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3], theta, speed, relative_yaw);

    ThisThread::sleep_for(15ms);
}

/**
 * @brief ps4コントローラーのボタンの入力を取得&機構制御
 */
void button_event() {
    while (true) {
        if (Triangle) {
            pwm[0] = SPEED_SCALE;
        } else if (Cross) {
            pwm[0] = -SPEED_SCALE;
        } else {
            pwm[0] = 0;
        }
        if (Up) {
            pwm[1] = SPEED_SCALE;
        } else if (Down) {
            pwm[1] = -SPEED_SCALE;
        } else {
            pwm[1] = 0;
        }
        if (R1) {
            pwm[2] = SPEED_SCALE;
        } else if (L1) { 
            pwm[2] = -SPEED_SCALE;
        } else {
            pwm[2] = 0;
        }
        if (R2) {
            pwm[3] = SPEED_SCALE;
        } else if (L2) {
            pwm[3] = -SPEED_SCALE;
        } else {
            pwm[3] = 0;
        }
        // if (Triangle) {
        //     DJI2.set_power(1, SPEED_SCALE);
        // } else if (Cross) {
        //     DJI2.set_power(1, -SPEED_SCALE);
        // } else {
        //     DJI2.set_power(1, 0);
        // }
        // if (Up) {
        //     DJI2.set_power(2, SPEED_SCALE);
        // } else if (Down) {
        //     DJI2.set_power(2, -SPEED_SCALE);
        // } else {
        //     DJI2.set_power(2, 0);
        // }
        // if (R1) {
        //     DJI2.set_power(3, SPEED_SCALE);
        // } else if (L1) {
        //     DJI2.set_power(3, -SPEED_SCALE);
        // } else {
        //     DJI2.set_power(3, 0);
        // }
        // if (R2) {
        //     DJI2.set_power(4, SPEED_SCALE);
        // } else if (L2) {
        //     DJI2.set_power(4, -SPEED_SCALE);
        // } else {
        //     DJI2.set_power(4, 0);
        // }
        ThisThread::sleep_for(10ms);
    }
}

/**
 * @brief can送信
 */
void c610_can_send() {
    while (true) {
        // printf("theta: %f, speed: %f, Triangle: %d, Cross: %d, Up: %d, Down: %d, R1: %d, L1: %d, R2: %d, L2: %d\n", theta, speed, Triangle, Cross, Up, Down, R1, L1, R2, L2);
        if (DJI1.send_message()) {
            // printf("%f, %f, %f, %f, %f, %f, %f\n", wheel_pid_output[0], wheel_pid_output[1], wheel_pid_output[2], wheel_pid_output[3], theta, speed, relative_yaw);
        } else {
            // printf("CAN1 Send Failed, ");
        }
        CANMessage msg(4, (const uint8_t*)pwm, 8);
        if (can2.write(msg)) {
        } else {
            // printf("CAN2 Send Failed\n");
        }
        printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", Triangle, Cross, Up, Down, R1, L1, R2, L2, pwm[0], pwm[1], pwm[2], pwm[3]);
        ThisThread::sleep_for(30ms);
    }
}


// main
int main() {
    // if (imu.begin()) {
    //     printf("BNO055 Initialized!\n");
    //     ThisThread::sleep_for(100ms);
    //     imu.update();
    //     yaw_offset = imu.getEuler().yaw;
    //     printf("Yaw offset set to: %.2f\n", yaw_offset);
    //     imu_available = true;
    // } else {
    //     printf("Failed to initialize BNO055. Running without IMU.\n");
    //     imu_available = false;
    // }

    // Thread imu_thread;
    // imu_thread.start(imu_polling);
    // Thread imu_read_thread;
    // imu_read_thread.start(imu_get_yaw);
    Thread serial_thread;
    serial_thread.start(serial_read);
    pc.set_blocking(false);
    Thread pid_thread;
    pid_thread.start(pid_control);
    Thread button_thread;
    button_thread.start(button_event);
    Thread c610_thread;
    c610_thread.start(c610_can_send);

    while (true) {
        // printf("%f\n", relative_yaw);
        
        // if (DJI1.send_message()) {
        //     printf("%f, %f, %f, %f, %f, %f, %f\n", wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3], theta, speed, relative_yaw);
        // } else {
        //     printf("CAN Send Failed\n");
        // }
                
        // ThisThread::sleep_for(30ms);
                
        
        // int16_t pwm_outputs[4];
        // for (int i = 0; i < 4; ++i) {
        //     pwm_outputs[i] = static_cast<int16_t>(SPEED_SCALE);
        // }
        // uint8_t data1[8] = {};
        // for (int i = 0; i < 4; ++i) {
        //     int16_t v = pwm_outputs[i];
        //     data1[i * 2]     = static_cast<uint8_t>((v >> 8) & 0xFF); // MSB
        //     data1[i * 2 + 1] = static_cast<uint8_t>(v & 0xFF);        // LSB
        // }
        
        // ↓は、not using pid

        // CANMessage can_msg(0x200, data1, sizeof(data1));
        // if (can1.write(can_msg)) {
        //     printf("%f, %f, %f, %f\n", wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
        // } else {
        //     // printf("CAN Send Failed\n");
        // }
    }
}