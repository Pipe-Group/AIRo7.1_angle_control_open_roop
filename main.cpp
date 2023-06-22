// Nucleo_CAN_slave (AIRo-2.3) angle control
// Created by Atsushi Kakogawa, 2019.09.19
// Department of Robotics, Ritsumeikan University, Japan
#include "mbed.h"
#include "CAN.h"

Ticker control; // timer for control

DigitalOut myled(PF_1);
PwmOut pwmA(PA_8);
PwmOut pwmB(PB_1);
AnalogIn potensio(PA_1);
float target_ang = 65.0 ,  def_ang=176.0, pote=176.0, //20230205ang 191 
e = 0.0 ,target_pwm, pote_fil, duty = 0.0;
int flag = 0; //0から1に変更

Timer t;

char tx_data[8];
char tx_ang1_U, tx_ang1_L, tx_ang2_U, tx_ang2_L, tx_tau, tx_now_ang ,tx_pot_U ,tx_pot_L;

// P-control timer
void controller() 
{
    if (pote < 330 && pote > 10)
    {
        if (flag == 1) {//angle control mode
         float kp=0.0 , ki=0, kd = 0;//kp=moto2
            if (target_ang < 150 && target_ang > 0) 
            {
                //pote_fil=0.9*pote_fil+0.1*pote;
                e = target_ang - pote*7/19;
                //I+= abs(e)*0.001;
                //duty= kp*abs(e) + ki*I;
                duty= kp*abs(e);
                //duty= kp*abs(e) + ki*abs(e)*dt;
                    if (e < 0) 
                    {
                        pwmA = 0;
                        pwmB = duty/100;
                    } 
                    else 
                    {
                        pwmA = duty/100;
                        pwmB = 0;
                    }
            }  
        } 
        else if (flag == 2) //Openroop
        {  
            
            duty = target_pwm;
            if (target_pwm < 51 && target_pwm > 0) //duty比が30以上だと危険
            {
                pwmA = 0;
                pwmB = abs(duty)/100;
            } 
            else 
            {
                pwmA = 0;
                pwmB = 0;
            }
        }
    }
    else
    {
            duty = target_pwm;
            pwmA = 0;
            pwmB = 0;
    }
}

int main() 
{
    control.attach(&controller, 0.001);  // 1 ms
    pwmA.period(0.02);  // 20 ms
    pwmB.period(0.02);  // 20 ms

    CAN can(PA_11, PA_12);
    can.frequency(100000);
    CANMessage msg;
   
    while(1) 
    {
        pote = potensio.read()*333.3;
        //pote_fil=0.9*pote_fil+0.1*pote;
        
        if(can.read(msg)) 
        {
            if (msg.data[0] == 1) // ID indentify (angle control mode)
            { 
                flag = 1;
                if (msg.data[1] == 0) // mode indentify (0: control,1: response, 2: PWM)
                { 
                    target_ang = (msg.data[2] << 8) + msg.data[3];
                } 
                else if (msg.data[1] == 1)  // mode indentify ( 1: response            0: control,)
                {
                    int i_ang1 = pote*100; //ang = potensio.read()*360; 
                    tx_ang1_U = (i_ang1 >> 8) & 0xff;
                    tx_ang1_L = i_ang1 & 0xff;
                    tx_data[0] = 1;             // ID
                    tx_data[1] = 1;             // mode (0: control, 1: response)
                    tx_data[2] = tx_ang1_U;      // response value upper 8bitは256なので360°表記出来ないから16bit使う
                    tx_data[3] = tx_ang1_L;      // response value lower 8bit
                    // tx_data[4] = tx_ang2_U;   //potensioの値としてコンバータに出力conL58
                    // tx_data[5] = tx_ang2_L;
                    //tx_data[4] = tx_pot_U;
                    //tx_data[5] = tx_pot_L;
                    //tx_data[6] = tx_tau;//start=0;
                    //tx_data[7] = tx_now_ang;//start=0;
                    can.write(CANMessage(1330, tx_data, 4));
                }
            } 
            else if (msg.data[0] == 5) // ID indentify (Open roop mode)
            { 
                flag = 2;
                if (msg.data[1] == 0)  // mode indentify (0: control, 1: response, 2: PWM)
                {
                    target_pwm = (msg.data[2] << 8) + msg.data[3];
                } 
                else if (msg.data[1] == 1)  // mode indentify (0: control, 1: response)
                {
                    int i_ang1 = pote*100;
                    tx_ang1_U = (i_ang1 >> 8) & 0xff;
                    tx_ang1_L = i_ang1 & 0xff; 
                    tx_data[0] = 5;             // ID
                    tx_data[1] = 1;             // mode (0: control, 1: response)
                    tx_data[2] = tx_ang1_U;      // response value upper 8bit
                    tx_data[3] = tx_ang1_L;      // response value lower 8bit
                    // tx_data[4] = tx_ang2_U;
                    // tx_data[5] = tx_ang2_L;
                    //tx_data[6] = tx_tau;//0;
                    //tx_data[7] = tx_now_ang;//0;
                    can.write(CANMessage(1330, tx_data, 4));
                }
            }
            myled =! myled; // LED is ON
            //wait (0.01);    // 10 msec
        }
        
        /*
        // CAN error check
        if(can.rderror() || can.tderror()){
            can.reset();
            wait (0.1);
        }
        */
    }
}
