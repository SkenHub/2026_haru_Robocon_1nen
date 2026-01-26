#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "asimawari_library/asimawari.h"
#include <cmath>

// --- 変数宣言 ---
Asimawari asimawari;
DebugData debugdata;

CanData can_data;
uint8_t sending_data[8], receiving_serial_data[8];
Gpio led;

// CubeMonitor監視用：必ず0で初期化
uint32_t debug_rx_count = 0;
uint32_t debug_last_id = 0;

double motor_target[4], motor_out[4],
    vx, vy, vz, speed, deg, deg_s, VX, VY;
double R_L = 519.417;
double wheel_radius = 100;

// --- 通信処理 (中継機からのデータを受け取る) ---
void communication() {
    if (can_data.rx_stdid != 0) {
        // 受信成功時にカウントアップ
        debug_rx_count++;
        debug_last_id = can_data.rx_stdid;

        // PCからの速度データ(0x160)を解析
        if (can_data.rx_stdid == 0x160) {

            int16_t raw_vx = (int16_t)((can_data.rx_data[0] << 8) | can_data.rx_data[1]);
            int16_t raw_vy = (int16_t)((can_data.rx_data[2] << 8) | can_data.rx_data[3]);
            int16_t raw_omega = (int16_t)((can_data.rx_data[4] << 8) | can_data.rx_data[5]);

            VX = raw_vx;
            VY = raw_vy;
            deg_s = raw_omega;
        }

        // 次の受信のためにIDをクリア
        can_data.rx_stdid = 0;
    }
}

// --- メイン制御処理 (モーターを動かす) ---
void main_interrupt() {
    vy = VY / 10  ;
    vx = VX / 10  ;
    vz = deg_s;

    // 4輪オムニホイールの制御実行
    asimawari.turn(omuni4, vx, vy, vz, R_L/2, wheel_radius);

    // エンコーダ値などの内部データを更新
    debugdata = asimawari.get_debug_data();
}

int main(void) {
    // システム初期化
    sken_system.init();

    // LEDを初期化（生存確認用）
    led.init(C13, INPUT_PULLUP);

    // --- モーターピン初期化 ---
    asimawari.mtr_pin_init(BR, Apin, B14, TIMER12, CH1);
    asimawari.mtr_pin_init(BR, Bpin, B15, TIMER12, CH2);
    asimawari.mtr_pin_init(BL, Apin, A8, TIMER1, CH1);
    asimawari.mtr_pin_init(BL, Bpin, A11, TIMER1, CH4);
    asimawari.mtr_pin_init(FR, Bpin, A6, TIMER13, CH1);
    asimawari.mtr_pin_init(FR, Apin, A7, TIMER14, CH1);
    asimawari.mtr_pin_init(FL, Bpin, B8, TIMER10, CH1);
    asimawari.mtr_pin_init(FL, Apin, B9, TIMER11, CH1);

    // --- エンコーダピン初期化 ---
    asimawari.enc_pin_init(FL, C6, C7, TIMER3, 100);
    asimawari.enc_pin_init(FR, B6, B7, TIMER4, 100);
    asimawari.enc_pin_init(BL, B3, A5, TIMER2, 100);
    asimawari.enc_pin_init(BR, A0, A1, TIMER5, 100);

    // --- PIDゲイン設定 ---
    asimawari.pid_set(FR, 15, 0, 0);
    asimawari.pid_set(FL, 15, 0, 0);
    asimawari.pid_set(BR, 15, 0, 0);
    asimawari.pid_set(BL, 15, 0, 0);

    // --- CAN2初期化 (中継機との接続用) ---
    // 配線: 中継機のB13/B12から足回り機のB13/B12へ
    sken_system.startCanCommunicate(B13, B12, CAN_2);
    sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data);

    // タイマー割り込み登録 (0:通信用1ms, 1:制御用10ms)
    sken_system.addTimerInterruptFunc(communication, 0, 1);
    sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);

    while (true) {
        // メインループ
    }
}
