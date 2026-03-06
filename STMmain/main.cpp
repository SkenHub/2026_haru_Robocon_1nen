#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Encoder encoder[4];
Encoder_data e_data[4];
Gpio limit[8];

CanData can_data_r;
uint8_t a[6],b[6],c[6],d[6],e[6],f[6],g[6];
double enc_data[4];
uint8_t send_data_limit[3]; // リミットスイッチのCAN送信データ

void main_interrupt(void) {
    // 各リミットスイッチの状態を読み取り
    for (int n = 0; n < 8; n++) {
        send_data_limit[n] = limit[n].read();
    }
}

void can(void){
	if(can_data_r.rx_stdid == 0x100){//canIDが0x201なら実行
		for(int m=0;m<8;m++){
		  c[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x101){//canIDが0x201なら実行
		for(int m=0;m<8;m++){
		  d[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x102){//canIDが0x200なら実行
		for(int m=0;m<8;m++){
		  e[m] = can_data_r.rx_data[m];
		}
    }
	if(can_data_r.rx_stdid == 0x160){//canIDが0x201なら実行
		for(int m=0;m<8;m++){
		  f[m] = can_data_r.rx_data[m];
		}
	}
}

int main(void) {
    // システム初期化
    sken_system.init();
    sken_system.startCanCommunicate(B13, B12, CAN_2); // CAN通信開始
    sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
    // リミットスイッチの初期化（各ピン、プルアップ設定）
    limit[0].init(B15, INPUT_PULLUP);
    limit[1].init(B14, INPUT_PULLUP);
    limit[2].init(A8, INPUT_PULLUP);
    limit[3].init(A7, INPUT_PULLUP);
    sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);
    sken_system.addTimerInterruptFunc(can, 3, 1);

    // メインループ：CAN通信でエンコーダ・リミットスイッチのデータを送信
    while (1) {
    	//sken_system.canTransmit(CAN_2, 0x300, send_data_enc, 8, 1);
    	sken_system.canTransmit(CAN_2, 0x300, send_data_limit, 8, 1);
    }
}
