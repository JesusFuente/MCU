
#include "PICSettings.h"
#include "dsPICDEM_MC1.h"
#include "PID.h"
#include "CAN.h"
#include <xc.h>
#include <dsp.h>

void PB_OnChange(char PBnum, char PBsts);   //Función de atención a las irq de pulsadores
void Driver_Fail();                         //Función de atención a la irq del Fallo Driver
void Encoder_Home();                        //Función de atención a la irq de Encoder en INDX
void CANrx();                               //Función de atención a la irq del CAN
void Blink();                               //Función de atención a la irq del Timer Blink

extern MOTOR_INFO Motor;
extern PID_DATA SpeedLoop;
extern PID_DATA PositLoop;

float posSP=0.0;
int blinking=0;



char FAIL;


int main(void)
{

SetupPorts();                  //Configura puertos
IniPB(PB_OnChange);            //Inicializa interrupcion pulsadores
IniLCD();                      //Inicializa LCD (Driver HD44780)
IniHall(0);                    //Inicializa control posicion Hall
IniPWM(Driver_Fail);           //Inicializa control PWM
IniQEI(Encoder_Home);          //Inicializa Encoder

IniCAN(CANrx);                 //Inicializa CAN
IniPositLoop();
iniTimerCAN();
iniTimerBlink(Blink);

//Reset Driver
while(!RstDriver());

StartPWM();
PositLoop.mode=PIDMODE_AUTO;
RstQEI();
      
WriteTxtLCD("s xxxx.x o xxx.x", 0 , 0);
WriteTxtLCD("posic: xxxx.x g ", 0 , 1);

while (1)
{
    if (!Motor.fault)
    {
    WriteNumLCD(posSP,6,1,2,0);
    if (Motor.direction) WriteNumLCD(-1*Motor.dcycle,5,1,11,0);
    else WriteNumLCD(Motor.dcycle,5,1,11,0);
    WriteNumLCD(Motor.position,6,1,7,1);
    }
}
return 0;
}

//*************************************************


void CANrx()
{  
    switch(CAN1RXSID){
        case 620:  //ID 620 correspon a comanda de consigna matlab
        {
            posSP=ReadCANChar()*POSPMAX;                        
            PIDSetSP(posSP,&PositLoop);
            break;
        }
        case 500:  //Matlab demands LEDs ON  or OFF    
        {
            switch(ReadCANChar()){
                case 0:  //Luces apagadas
                {
                    LED_D6=0;
                    LED_D7=0;
                    break;
                }
                case 1:  //Luces cortas
                {
                    LED_D6=1;
                    LED_D7=0;
                    break;
                }
                case 3:  //Luces cortas
                {
                    LED_D6=1;
                    LED_D7=1;
                    break;
                }
            }
            break;
        }
        case 550: //Matlab demands LED BLINKING
        {
            blinking=ReadCANChar();
            break;
        }
    } 
}

//*************************************************

void PB_OnChange(char PBnum, char PBsts)
{
    if (PBsts==1)
    {
        switch(PBnum){
            case 4:
            {
                RstDriver();
                StartPWM();
                PositLoop.mode=PIDMODE_AUTO;
                break;
            }
            case 5:
            {
                break;
            }
            case 6:
            {
                break;
            }
            case 7:
            {          
                break;
            }
            
        }
                
    }
}

//*************************************************

void Blink()
{
    switch(blinking){
        case 0:  //OFF
            {
                LED_D8=0;
                break;
            }
        case 1:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                }
                else
                {
                    LED_D8=0;
                }
                break;
            }
        case 2:  //Right
            {
                if (LED_D8==0) LED_D8=1;
                else LED_D8=0;
                break;
            }
        case 3:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                }
                else
                {
                    LED_D8=0;
                }
                break;
            }
        case 4:  //Left
            {
                LED_D8=0;
                break;
            }
        case 5:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                }
                else
                {
                    LED_D8=0;
                }
                break;
            }
    }
}

//*************************************************

void Driver_Fail(){}                         //Función de atención a la irq del Fallo Driver
void Encoder_Home(){}                        //Función de atención a la irq de Encoder en INDX