
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "asimawari_library/asimawari.h"
#include <cmath>

Asimawari asimawari;
DebugData debugdata;

CanData can_data;
uint8_t sending_data[8],receiving_serial_data[8];//通信データ
Gpio led;

double motor_target[4],motor_out[4],
	vx,vy,vz,speed,deg,deg_s,VX,VY;
double R_L = 519.417, //旋回直径
	   wheel_radius= 100;//タイヤ直径

void communication(){
	if (can_data.rx_stdid == 0x160)
	        {
	            // Python側の pack('>hhh') = Big Endian, 16bit signed int x 3 に対応

	            // Vx の復元 (Byte 0, 1)
	            // 上位バイトを8bitシフトして下位バイトと結合
	            int16_t raw_vx = (int16_t)((can_data.rx_data[0] << 8) | can_data.rx_data[1]);

	            // Vy の復元 (Byte 2, 3)
	            int16_t raw_vy = (int16_t)((can_data.rx_data[2] << 8) | can_data.rx_data[3]);

	            // Omega の復元 (Byte 4, 5)
	            int16_t raw_omega = (int16_t)((can_data.rx_data[4] << 8) | can_data.rx_data[5]);

	            // 5. スケーリングを元に戻す (/10.0)
	            VX = (double)raw_vx / 10.0;
	            VY = (double)raw_vy / 10.0;
	            deg_s = (double)raw_omega / 10.0;

	            // 受信処理完了後、IDをクリアして二重読み込みを防止（任意ですが推奨）
	            can_data.rx_stdid = 0;
	        }

	sending_data[0] = led.read();
	sken_system.canTransmit(CAN_2,0x200,sending_data,8,0);
}

void main_interrupt(){
	vx = VX;//speed*cos(deg);
	vy = VY;//speed*sin(deg);
	vz = deg_s;
	debugdata = asimawari.get_debug_data();
	asimawari.turn(omuni4,vx,vy,vz,R_L/2,wheel_radius);
}

int main(void)
{
	sken_system.init();
	led.init(C13,INPUT_PULLUP);

	asimawari.mtr_pin_init(FR,Apin,B14,TIMER12,CH1);
	asimawari.mtr_pin_init(FR,Bpin,B15,TIMER12,CH2);

	asimawari.mtr_pin_init(FL,Apin,A8,TIMER1,CH1);
	asimawari.mtr_pin_init(FL,Bpin,A11,TIMER1,CH4);

	asimawari.mtr_pin_init(BR,Apin,A6,TIMER3,CH1);
	asimawari.mtr_pin_init(BR,Bpin,A7,TIMER3,CH2);

	asimawari.mtr_pin_init(BL,Apin,B8,TIMER10,CH1);
	asimawari.mtr_pin_init(BL,Bpin,B9,TIMER11,CH1);

	asimawari.enc_pin_init(FR,C6,C7,TIMER8,100);
	asimawari.enc_pin_init(FL,B6,B7,TIMER4,100);
	asimawari.enc_pin_init(BR,B3,A5,TIMER2,100);
	asimawari.enc_pin_init(BL,A0,A1,TIMER5,100);

	asimawari.pid_set(FR,10,0,0);
	asimawari.pid_set(FL,10,0,0);
	asimawari.pid_set(BR,10,0,0);
	asimawari.pid_set(BL,10,0,0);

	sken_system.startCanCommunicate(B13, B12, CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data);

	sken_system.addTimerInterruptFunc(communication,0,1);
	sken_system.addTimerInterruptFunc(main_interrupt,1,1);
	while(true){
	}
}
