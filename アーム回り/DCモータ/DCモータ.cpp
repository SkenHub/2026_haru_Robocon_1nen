
//LEDライトプログラム追加
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

// インスタンスの準備
Motor motor1, motor2, motor3, light;
Encoder encoder;
Encoder_data e_data;
CanData can_data;

// デバッグ・制御用変数
Pid pid_control;

uint8_t can100_rx[8];      // 0x100（操作系：モーター1,2,3）
uint8_t can101_rx[8]; // 0x102（操作系：LEDライト）
uint8_t can102_rx[8];
uint8_t can160_rx[8];
uint8_t can300_rx[8];
uint8_t limit_rx[8];    // 0x300（リミット系：A, B, C）
double target = 100;  // 目標値（例えば目標速度）
double now = 0;       // 現在の値（現在の速度）
double out = 0;       // 出力（モータへの指令など）
int last_b = 0;         // モーター3のエッジ判定用
int count1 = 0;
double deg;

// 毎1msで呼び出される関数
void pid_func() {
    int now_b = can100_rx[2];
    if (last_b == 0 && now_b == 1) {
    	count1++; // 押された瞬間にカウントアップ
    }
    last_b = now_b;

	if (count1 % 2 != 0){
		target = 90;
	}
	else{
		target = 10;
	}
		out = pid_control.control(target, now, 1);  // 目標値と現在値から制御出力を計算
}


// タイマー割り込み関数（毎ms呼び出される）
void encoder_interrupt() {
    encoder.interrupt(&e_data);  // エンコーダの値を更新
}


// CAN受信処理
void can_check(void) {
    if (can_data.rx_stdid == 0x100) {
        for (int i = 0; i < 8; i++) {
            can100_rx[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x102) {
        for (int i = 0; i < 8; i++) {
            can102_rx[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x101) {
        for (int i = 0; i < 8; i++) {
            can101_rx[i] = can_data.rx_data[i];
        }
    }
    if (can_data.rx_stdid == 0x160) {
        for (int i = 0; i < 8; i++) {
            can160_rx[i] = can_data.rx_data[i];
        }
    }
    /*if (can_data.rx_stdid == 0x300) {
        for (int i = 0; i < 8; i++) {
            can300_rx[i] = can_data.rx_data[i];
        }
    }*/
    can_data.rx_stdid = 0;

}




int main(void) {
    sken_system.init();
    //sken_system.addTimerInterruptFunc(encoder_interrupt, 0, 1); // タイマー割り込み1ms周期で登録
    pid_control.setGain(0.8, 0, 0);  // 適当なゲイン設定
    sken_system.addTimerInterruptFunc(pid_func, 0, 1);  // 1msごとにPID制御実行

    // CAN2開始 (B13, B12)
    sken_system.startCanCommunicate(B13, B12, CAN_2);
    sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data);

    // --- モーター初期化 (ALTAIR_MDD_V2スタイル) ---
    // 伸縮: B8, B9
    motor1.init(Apin, A6, TIMER13, CH1);
    motor1.init(Bpin, A7, TIMER14, CH1);

    // 昇降: A8, A11
    motor2.init(Apin, A8, TIMER1, CH1);
    motor2.init(Bpin, A11, TIMER1, CH4);

    // 櫓持ち上げ: B14, B15
    motor3.init(Apin, B14, TIMER12, CH1);
    motor3.init(Bpin, B15, TIMER12, CH2);

    // LEDライト (light): A6, A7
    light.init(Apin, B8, TIMER10, CH1);
    light.init(Bpin, B9, TIMER11, CH1);

    encoder.init(A0, A1, TIMER5);

    while (1) {
        can_check(); // 全IDのCANデータを更新
        encoder.interrupt(&e_data);
        deg = e_data.deg;
        now = deg;
        double angle = e_data.deg;      // 現在の角度 [度]
        double velocity = e_data.volcity; // 現在の速度 [mm/s]

        // --- リミット情報の取得 (0x300) ---
        int A = can300_rx[0];
        int B = can300_rx[1];
        int C = can300_rx[2];

        // --- Motor1 制御 ---
        if (can101_rx[1] == 1 && A == 0) motor1.write(50);
        else if (can101_rx[4] == 1 && B == 0) motor1.write(-50);
        else motor1.write(0);

        // --- Motor2 制御 ---
        if (can100_rx[5] == 1 ) motor2.write(30);
        else if (can100_rx[6] == 1) motor2.write(-30);
        else motor2.write(0);

        // --- Motor3 制御 (エッジ判定トグル) ---
/*        int now_b = can_rx[2];
        if (last_b == 0 && now_b == 1) {
        	count1++; // 押された瞬間にカウントアップ
        }
        last_b = now_b;

        if (count1 % 2 != 0) {
        // 奇数：垂直(90度)へ移動
        	if (deg < 90) motor3.write(30);
        	else motor3.write(0);
        } else {
        // 偶数：水平(0度)へ移動
        	if (deg > 1) motor3.write(-30);
        	else motor3.write(0);
        }*/

        	motor3.write(out);



        // --- LEDライト制御 (0x102の3番目 = yazirusi_rx[2]) ---
        if (can102_rx[2] == 1) {
            light.write(100); // 点灯
        } else {
            light.write(0);   // 消灯
        }
    }
}
