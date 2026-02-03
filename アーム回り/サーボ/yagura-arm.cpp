#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"


Gpio sw;
uint8_t can_rx[1]; // can受信データ格納
CanData can_data;
int count = 0;
int count1 = 0;
int count2 = 0;


RcPwm servo;
RcPwm servo2;
RcPwm servo3;

void can_kakunou(void)
{
    if (can_data.rx_stdid == 0x160)
    {
        can_rx[1] = can_data.rx_data[1];
    }

}


int main(void) // メイン関数
{
    sken_system.init();              // マイコンのsken初期設定

    sken_system.startCanCommunicate(B13, B12, CAN_2);        // CANピン設定
    sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data); // CAN受信


    sw.init(C13, INPUT_PULLUP);             // swのピン設定(STM搭載のswはC13ピン)

    servo.init(A5, TIMER2, CH1);  // A5ピンをサーボ信号出力に設定
    servo2.init(A6, TIMER2, CH2);
    servo3.init(A7, TIMER2, CH3);

    while (1) // 無限ループ
    {

        if (can_data.rx_stdid == 0x100)
        {
            count = count + 1;
            if (count % 2 == 1)
            {
                servo.turn(180);
            }
            else
            {
                servo.turn(90);
            }
        }

        if(can_data.rx_stdid == 0x100)
        {
            count1 = count1 + 1;
            if (count1 % 2 == 1)
             {
                servo2.turn(180);
            }
            else
            {
                servo2.turn(90);
            }
        }


        if(can_data.rx_stdid == 0x100)
        {
            count2 = count2 + 1;
            if (count2 % 2 == 1)
            {
                servo3.turn(180);
            }
            else
            {
                servo3.turn(90);
            }
        }

    }
}