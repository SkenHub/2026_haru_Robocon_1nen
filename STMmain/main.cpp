#include "stm32f4xx.h"
#include "sken_library/include.h"
#include <string.h>

// 1. 受信用構造体の宣言
CanData rx_data;

// 前回の値を保持（PA11のバタつきを抑えるための停止判定用）
int16_t last_vx = 0, last_vy = 0, last_omega = 0;

int main(void) {
    sken_system.init();

    // CAN1: PC側 (A12, A11) / CAN2: 足回り側 (B13, B12)
    sken_system.startCanCommunicate(A12, A11, CAN_1);
    sken_system.startCanCommunicate(B13, B12, CAN_2);

    // ライブラリに実在する関数で受信構造体を登録
    sken_system.addCanRceiveInterruptFunc(CAN_1, &rx_data);

    Gpio limit_sw;
    limit_sw.init(C13, INPUT_PULLUP);
    uint32_t last_limit_time = 0;

    while (1) {
        // --- IDグループ別の個別リレー処理 ---
        // 構造体の rx_stdid をチェックすることで受信を検知
        uint32_t id = rx_data.rx_stdid;

        if (id != 0) {
            // 受信したIDごとに「完全に別」として処理
            switch (id) {
                case 0x160: // 【グループA：速度データ】
                    {
                        int16_t vx = (int16_t)((rx_data.rx_data[0] << 8) | rx_data.rx_data[1]);
                        int16_t vy = (int16_t)((rx_data.rx_data[2] << 8) | rx_data.rx_data[3]);
                        int16_t omega = (int16_t)((rx_data.rx_data[4] << 8) | rx_data.rx_data[5]);

                        // 停止時かつ前回も停止なら送信スキップ（PA11のノイズをカット）
                        if (vx == 0 && vy == 0 && omega == 0 && last_vx == 0 && last_vy == 0 && last_omega == 0) {
                            // 送信しない
                        } else {
                            sken_system.canTransmit(CAN_2, 0x160, rx_data.rx_data, 8);
                        }
                        last_vx = vx; last_vy = vy; last_omega = omega;
                    }
                    break;

                case 0x100: // 【グループB：ボタン系】
                case 0x101:
                case 0x102:
                    sken_system.canTransmit(CAN_2, id, rx_data.rx_data, 8);
                    break;

                default:
                    // 想定外のIDは無視
                    break;
            }
            // 重要：処理が終わったら即座に0に戻し、多重送信を防ぐ
            rx_data.rx_stdid = 0;
        }

        // --- グループC：リミットスイッチ 0x200 ---
        // 受信とは独立したタイマーで送信
        if (sken_system.millis() - last_limit_time >= 20) {
            uint8_t msg = limit_sw.read() ? 0 : 1;
            sken_system.canTransmit(CAN_2, 0x200, &msg, 1);
            last_limit_time = sken_system.millis();
        }
    }
}
/*STMmain
#include "stm32f4xx.h"
#include "sken_library/include.h"
#include <string.h>

CanData can_data;
Gpio limit_sw;

// --- 中継デバッグ用変数 ---
volatile uint32_t relay_rx_count = 0; // PC(CAN1)からの受信数
volatile uint32_t relay_tx_count = 0; // 足回り(CAN2)への送信試行数
volatile uint32_t relay_last_id = 0;

int main(void) {
    sken_system.init();
    // CAN1: PC(A12,A11) / CAN2: 足回り(B13,B12)
    sken_system.startCanCommunicate(A12, A11, CAN_1);
    sken_system.startCanCommunicate(B13, B12, CAN_2);
    sken_system.addCanRceiveInterruptFunc(CAN_1, &can_data);

    limit_sw.init(C13, INPUT_PULLUP);
    uint32_t last_send_time = 0;
    uint8_t temp_data[8];

    while (1) {
        uint32_t received_id = can_data.rx_stdid;
        if (received_id != 0) {
            can_data.rx_stdid = 0;
            memcpy(temp_data, can_data.rx_data, 8);

            relay_rx_count++;
            relay_last_id = received_id;

            if (received_id == 0x160 || (received_id >= 0x100 && received_id <= 0x102)) {
                // 送信処理
                sken_system.canTransmit(CAN_2, received_id, temp_data, 8);
                relay_tx_count++;
            }
        }

        if (sken_system.millis() - last_send_time >= 10) {
            uint8_t limit_msg = limit_sw.read() ? 0 : 1;
            sken_system.canTransmit(CAN_2, 0x200, &limit_msg, 1);
            last_send_time = sken_system.millis();
        }
    }
}
*/


