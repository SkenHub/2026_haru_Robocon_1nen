#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Motor motor1;
Motor motor2;
Motor motor3;

Encoder encoder;
Encoder_data e_data;

int count;
double rot, deg, distance, volcity, rps;

CanData can_data;
uint8_t can_rx[8];

int a;
int b;
int count1;

void can_kakunou(void)
{
	if(can_data.rx_stdid == 0x100){
		for(int a=0;a>8;a++){
		can_rx[a] = can_data[a];
		}
	}
}
int main(void)
{
	sken_system.init();

	sken_system.startCanCommunicate(B13,B12,CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data);
	sken_system.addTimerInterruptFunc(can_kakunou, 3, 1);

	motor1.init(Apin, B8, TIMER2, CH1);
	motor1.init(Bpin, B9, TIMER2, CH2);
	motor2.init(Apin, B8, TIMER2, CH1);
	motor2.init(Bpin, B9, TIMER2, CH2);
	motor3.init(Apin, B8, TIMER2, CH1);
	motor3.init(Bpin, B9, TIMER2, CH2);

	encoder.init(A0, A1, TIMER5);

	while(1)
	{
		encoder.interrupt(&e_data);
		deg = e_data.deg;

		if(can_rx[0] == 1){
			motor1.write(10);
		}
		else if(can_rx[1]== 1){
			motor1.write(-10);
		}
		else{
			motor1.write(0);
		}

		if(can_rx[2] == 1){
			motor2.write(10);
		}
		else if(can_rx[3]== 1){
			motor2.write(-10);
		}
		else{
			motor2.write(0);
		}
		a = b;//a = b aにbを代入
		if(can_rx[4] == 1){
			b = 1;
		}
		else{
			b = 0;
		}//(!sw.read())? もしスイッチが押されたら〔?はifの省略、前の()が条件〕  b = 1　bに1を代入  :b = 0 でなければbに0を代入〔:はelseの省略〕
		if(a == 0&&b == 1)count1++;//if(a == false&&b == true) もしaが偽(0)かつbが真(1)ならば  c++ cに1足す
		if(count1%2 == 0){
			if(deg<90){
				motor3.write(10);
			}
			else{
				motor3.write(0);
			}
		}
		else if(count1 % 2 != 0){
			if(deg>1){
				motor3.write(-deg-5);
			}
			else{
				motor3.write(0);
			}
		}
		else{
			motor3.write(0);
		}
	}
}