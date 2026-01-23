#include "stm32f4xx.h"
#include "sken_library/include.h"

// --- オブジェクト宣言 ---
Motor motor_FL, motor_FR, motor_BL, motor_BR;
Encoder encoder_FL, encoder_FR, encoder_BL, encoder_BR;
Encoder_data e_data_FL, e_data_FR, e_data_BL, e_data_BR;
Pid pid_FL, pid_FR, pid_BL, pid_BR;

CanData relay_can_data;

// --- 制御用変数 ---
// 目標値 (各輪の目標速度 mm/s)
volatile double target_FL = 0, target_FR = 0, target_BL = 0, target_BR = 0;
// PID出力
double out_FL = 0, out_FR = 0, out_BL = 0, out_BR = 0;

// ロボット定数 (機体に合わせて調整)
const double R_L = 519.417; // 旋回直径
const double R = R_L / 2.0; // ロボット中心から車輪までの距離

// --- デバッグ監視用 ---
volatile uint32_t rx_count = 0;
volatile int16_t debug_vx = 0, debug_vy = 0, debug_omega = 0;

// 1msごとに呼ばれる制御・通信解析ループ
void control_loop() {
    // 1. CAN受信解析
    if (relay_can_data.rx_stdid == 0x160) {
        rx_count++;
        // データの復元 (Big Endianを想定)
        int16_t vy = (int16_t)((relay_can_data.rx_data[0] << 8) | relay_can_data.rx_data[1]);
        int16_t omega = (int16_t)((relay_can_data.rx_data[2] << 8) | relay_can_data.rx_data[3]);
        int16_t vx = (int16_t)((relay_can_data.rx_data[4] << 8) | relay_can_data.rx_data[5]);

        vx = vx / 10;
        vy = vy / 10;
        debug_vx = vx; debug_vy = vy; debug_omega = omega;

        // 2. 逆運動学計算 (目標速度 mm/s への分解)
        // 配置: 前左(FL), 前右(FR), 後右(BR), 後左(BL) の順
        target_FL = (double)( vx + vy + (omega * R / 100.0));
        target_FR = (double)(-vx + vy + (omega * R / 100.0));
        target_BR = (double)(-vx - vy + (omega * R / 100.0));
        target_BL = (double)( vx - vy + (omega * R / 100.0));

        relay_can_data.rx_stdid = 0; // 受信リセット
    }

    // 3. エンコーダ更新
    encoder_FL.interrupt(&e_data_FL);
    encoder_FR.interrupt(&e_data_FR);
    encoder_BL.interrupt(&e_data_BL);
    encoder_BR.interrupt(&e_data_BR);

    // 4. PID計算 (目標速度と現在速度 e_data.volcity で計算)
    // +35はフィードフォワード(動き出しの補助)
    out_FL = pid_FL.control(target_FL, e_data_FL.volcity, 1) + (target_FL > 0 ? 35 : (target_FL < 0 ? -35 : 0));
    out_FR = pid_FR.control(target_FR, e_data_FR.volcity, 1) + (target_FR > 0 ? 35 : (target_FR < 0 ? -35 : 0));
    out_BL = pid_BL.control(target_BL, e_data_BL.volcity, 1) + (target_BL > 0 ? 35 : (target_BL < 0 ? -35 : 0));
    out_BR = pid_BR.control(target_BR, e_data_BR.volcity, 1) + (target_BR > 0 ? 35 : (target_BR < 0 ? -35 : 0));
}

int main(void) {
    sken_system.init();

    // --- ピン初期化 (提供された設定をそのまま適用) ---
    motor_FL.init(Apin, B8, TIMER10, CH1); motor_FL.init(Bpin, B9, TIMER11, CH1);
    encoder_FL.init(C6, C7, TIMER3); // BLのコードと混同注意：提供ピンに合わせました

    motor_FR.init(Apin, A6, TIMER13, CH1); motor_FR.init(Bpin, A7, TIMER14, CH1);
    encoder_FR.init(B6, B7, TIMER4);

    motor_BL.init(Apin, A8, TIMER1, CH1); motor_BL.init(Bpin, A11, TIMER1, CH4);
    encoder_BL.init(B3, A5, TIMER2);

    motor_BR.init(Apin, B14, TIMER12, CH1); motor_BR.init(Bpin, B15, TIMER12, CH2);
    encoder_BR.init(A0, A1, TIMER5);

    // --- 制御設定 ---
    pid_FL.setGain(0.01, 0.0, 0);
    pid_FR.setGain(0.01, 0.0, 0);
    pid_BL.setGain(0.01, 0.0, 0);
    pid_BR.setGain(0.01, 0.0, 0);

    // --- CAN通信設定 (足回り側: CAN1 A12/A11) ---
    sken_system.startCanCommunicate(A12, A11, CAN_1);
    sken_system.addCanRceiveInterruptFunc(CAN_1, &relay_can_data);

    // 周期割り込み開始
    sken_system.addTimerInterruptFunc(control_loop, 0, 1); // 1ms

    while (1) {
        // 各輪の回転方向を Motor.write で反映
        // 提供コードの - / + の向きを継承
        motor_FL.write(-out_FL);
        motor_FR.write(out_FR);
        motor_BL.write(-out_BL);
        motor_BR.write(out_BR);
    }
}
