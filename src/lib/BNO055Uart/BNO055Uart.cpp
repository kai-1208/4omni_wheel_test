#include "BNO055Uart.hpp"

// コンストラクタ: シリアルポートを初期化
BNO055Uart::BNO055Uart(PinName tx, PinName rx) : _serial(tx, rx, 115200) {
    // 内部変数を初期化
    _euler = {0.0f, 0.0f, 0.0f};
}

// センサーの初期化
bool BNO055Uart::begin(chrono::milliseconds timeout) {
    Timer t;
    t.start();

    // 受信バッファをクリア
    char dummy;
    while(_serial.readable()) _serial.read(&dummy, 1);

    while (t.elapsed_time() < timeout) {
        // 1. CONFIGモードに設定
        if (!reg_write(OPR_MODE, 0x00)) {
            continue; // 失敗したらリトライ
        }
        ThisThread::sleep_for(20ms); // モード変更のための待機

        // 2. IMUモードに設定
        if (!reg_write(OPR_MODE, 0x08)) {
            continue; // 失敗したらリトライ
        }
        ThisThread::sleep_for(10ms);

        return true; // 成功
    }
    return false; // タイムアウト
}

// データの更新
bool BNO055Uart::update() {
    uint8_t buffer[6];
    if (reg_read(EUL_DATA_X_LSB, buffer, 6)) {
        // リトルエンディアンで送られてくるデータを16bit整数に組み立てる
        int16_t bno_x_raw = (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);
        int16_t bno_y_raw = (int16_t)(((uint16_t)buffer[3] << 8) | (uint16_t)buffer[2]);
        int16_t bno_z_raw = (int16_t)(((uint16_t)buffer[5] << 8) | (uint16_t)buffer[4]);

        // データシートの仕様に従い、物理単位（度）に変換
        float bno_roll = (float)bno_x_raw / 16.0f;
        float bno_pitch = (float)bno_y_raw / 16.0f;
        float bno_yaw = (float)bno_z_raw / 16.0f;

        // yawとrollを入れ替え
        _euler.roll = bno_yaw;
        _euler.pitch = bno_pitch;
        _euler.yaw = bno_roll;

        return true;
    }
    _euler = {0.0f, 0.0f, 0.0f};
    return false;
}

// 内部で保持しているオイラー角データを返す
BNO055Uart::EulerAngles BNO055Uart::getEuler() const {
    return _euler;
}

// タイムアウト付きで指定バイト数を読み込むヘルパー関数
static bool read_with_timeout(UnbufferedSerial& serial, void* buffer, size_t length, chrono::milliseconds timeout) {
    uint8_t* ptr = static_cast<uint8_t*>(buffer);
    Timer t;
    t.start();
    for (size_t i = 0; i < length; i++) {
        while (!serial.readable()) {
            if (t.elapsed_time() > timeout) {
                return false; // タイムアウト
            }
        }
        serial.read(ptr + i, 1);
    }
    return true;
}

// 受信バッファをクリアするヘルパー関数
static void flush_serial_buffer(UnbufferedSerial& serial) {
    char dummy;
    while(serial.readable()) {
        serial.read(&dummy, 1);
    }
}

// レジスタへの書き込み
bool BNO055Uart::reg_write(uint8_t addr, uint8_t data) {
    // ★★ 通信前に必ずバッファをクリア ★★
    flush_serial_buffer(_serial);

    char tx_buffer[] = {0xAA, 0x00, addr, 0x01, data};
    char rx_buffer[2];

    _serial.write(tx_buffer, sizeof(tx_buffer));
    
    if (read_with_timeout(_serial, rx_buffer, sizeof(rx_buffer), 20ms)) {
        // 応答が0xEE 0x01 (WRITE_SUCCESS)なら成功
        return (rx_buffer[0] == (char)0xEE && rx_buffer[1] == (char)Response::SUCCESS);
    }
    return false;
}

// レジスタからの読み込み
bool BNO055Uart::reg_read(uint8_t addr, uint8_t* buffer, uint8_t len) {
    // ★★ 通信前に必ずバッファをクリア ★★
    flush_serial_buffer(_serial);

    char tx_buffer[] = {0xAA, 0x01, addr, len};
    char rx_header[2];

    _serial.write(tx_buffer, sizeof(tx_buffer));

    if (read_with_timeout(_serial, rx_header, sizeof(rx_header), 20ms)) {
        // 応答ヘッダが0xBB (Read Success)で、データ長が一致していれば成功
        if (rx_header[0] == (char)0xBB && rx_header[1] == len) {
            // 続けてデータ本体を受信
            if (read_with_timeout(_serial, buffer, len, 20ms)) {
                return true;
            }
        }
    }
    return false;
}