#include "stm32f4xx.h"
#include "sken_library/include.h"
// --- CubeMonitorで監視するためのグローバル変数 ---
uint32_t can_debug_count = 0;    // 0x101を受信した回数 (CAN確認用)
uint8_t  monitor_rx0 = 0;       // 受信した生のデータ0 (R2)
uint8_t  monitor_rx4 = 0;       // 受信した生のデータ4 (L2)
uint8_t  monitor_rx3 = 0;

double   servo_val1 = 50.0;     // サーボ1に今出している命令値(%)
double   servo_val2 = 50.0;     // サーボ2に今出している命令値(%)
double   servo_val3 = 50.0;     // サーボ3に今出している命令値(%)

int servo_state0 = 0;           // 0 or 1
int servo_state1 = 0;
int servo_state2 = 0;
int last_b = 0;         // モーター3のエッジ判定用
int count1 = 0;

uint8_t last_button0 = 0;
uint8_t last_button1 = 0;
uint8_t last_button2 = 0;
uint8_t B[8];
uint8_t A[8];
uint8_t C[8];
uint8_t D[8];
uint8_t  can100_rx[8];



// ----------------------------------------------

Gpio sw;
CanData can_data;
RcPwm servo1, servo2, servo3;

// CAN受信処理
void can_check(void) {
    if (can_data.rx_stdid == 0x100) {
        for (int i = 0; i < 8; i++) {
            can100_rx[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x102) {
        for (int i = 0; i < 8; i++) {
            A[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x101) {
        for (int i = 0; i < 8; i++) {
            D[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x160) {
        for (int i = 0; i < 8; i++) {
            B[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x300) {
        for (int i = 0; i < 8; i++) {
            C[i] = can_data.rx_data[i];
        }
    }
    can_data.rx_stdid = 0;

}

int main(void)
{
    sken_system.init();
    sken_system.startCanCommunicate(B13,B12, CAN_2);
    sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data);

    sw.init(C13, INPUT_PULLUP);
    servo1.init(B7, TIMER4, CH2);
    servo2.init(C6, TIMER3, CH1);
    servo3.init(C8, TIMER8, CH3);

    while (1) {
        can_check(); // 全IDのCANデータを更新
                // 生のデータをモニター用変数にコピー
                monitor_rx0 = D[3];
                monitor_rx4 = D[0];
                monitor_rx3 = can100_rx[0];
                // --- サーボ左の処理 ---
                if (last_button0 == 0 && monitor_rx0 == 1) {
                    servo_state0 = !servo_state0;
                    servo_val1 = (servo_state0 ? 100.0 : 60.0);
                    servo1.turn(servo_val1);
                }
                last_button0 = monitor_rx0;

                // --- サーボ右の処理 ---
                if (last_button1 == 0 && monitor_rx4 == 1) {
                    servo_state1 = !servo_state1;
                    servo_val2 = (servo_state1 ? 110.0 : 50.0);
                    servo2.turn(servo_val2);
                }
                last_button1 = monitor_rx4;

                //　--- サーボリングの処理　---

                /*if (last_button2 == 0 && monitor_rx3 == 1) {
                    servo_state2 = !servo_state2;
                    servo_val3 = (servo_state2 ? 210.0 : -30);
                    servo3.turn(servo_val3);
                }
                last_button2 = can100_rx[3];*/

                // --- サーボ3 (提案されたカウントアップ方式) ---
                int now_b = can100_rx[0]; // 0x100のデータindex 2を使用

                // 押された瞬間にカウントアップ（エッジ検出）
                if (last_b == 0 && now_b == 1) {
                	count1++;
                }
                last_b = now_b;
                monitor_rx3 = now_b; // モニター用

                // カウント数に応じてターゲットを決定
                if (count1 % 2 != 0) {
                	servo_val3 = 210; // 奇数回目
                }
                else {
                	servo_val3 = -30; // 偶数回目
                }

                // サーボを動かす
                servo3.turn(servo_val3);

                // チャタリング防止のための短い待機
               // sken_system.delayMillis(1);
     }
}


