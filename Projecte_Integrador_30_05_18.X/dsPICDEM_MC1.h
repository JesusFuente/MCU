#include <xc.h>

//Frecuencia trabajo
//#define FCY  14745600   // xtal = 7.3728 Mhz; PLLx8 FCY=Fosc/4
#define FCY  16000000   // xtal = 8.0 Mhz; PLLx8 FCY=Fosc/4

//LEDs
#define LED_D6	LATAbits.LATA9
#define LED_D7	LATAbits.LATA10
#define LED_D8	LATAbits.LATA14
#define LED_D9	LATAbits.LATA15

//Pulsadores
#define PBUTTON_S4	PORTGbits.RG6
#define PBUTTON_S5	PORTGbits.RG7
#define PBUTTON_S6	PORTGbits.RG8
#define PBUTTON_S7	PORTGbits.RG9

//LCD
#define LCDRS   LATCbits.LATC3      //LCD Reg. Select
#define LCDENA  LATDbits.LATD13     //LCD enable data
#define LCDRW   LATCbits.LATC1      //LCD Read/write
#define LCDD0   PORTDbits.RD0       //LCD DB4
#define LCDD1   PORTDbits.RD1       //LCD DB5
#define LCDD2   PORTDbits.RD2       //LCD DB6
#define LCDD3   PORTDbits.RD3       //LCD DB7
// Retardos LCD aplicables a funcion wait()
#define Delay_200ms (FCY * 0.200000/14)
#define Delay_20ms  (FCY * 0.020000/14)
#define Delay_5ms   (FCY * 0.005000/14)
#define Delay_10ms  (FCY * 0.010000/14)
#define Delay_1ms   (FCY * 0.001000/14)
#define Delay_200us (FCY * 0.000200/14)
#define Delay_5us   (FCY * 0.000005/14)

//HALL
#define CAP1  PORTDbits.RD8
#define CAP2  PORTDbits.RD9
#define CAP3  PORTDbits.RD10

//PWM
#define FPWM        16000             //Fecuencia PWM 16 KHz
#define PHA1L       LATEbits.LATE0
#define PHA1H       LATEbits.LATE1
#define PHA2L       LATEbits.LATE2
#define PHA2H       LATEbits.LATE3
#define PHA3L       LATEbits.LATE4
#define PHA4H       LATEbits.LATE5
#define DRVFAIL     !PORTEbits.RE8    //Fallo Driver
#define DRVRST      LATEbits.LATE9    //Reset Fallo Driver
#define PFC         LATDbits.LATD5
#define Delay_RST   (FCY * 0.05/14)   //Retardo Reset Driver

//ENCODER
#define PPR         250.0             //Pulsos por revolucion
#define RPMMAX      4000.0            //RPM maximas (calc veloc)
#define IPR         1000.0            //Interrup. por revolucion
#define SPESTM      60.0/RPMMAX       //Tiempo muestreo velocidad (seg)
#define LED_FR      LATDbits.LATD7    //LED FWD/REV
#define QEINDX      PORTBbits.RB3     //INDX
#define QEA         PORTBbits.RB4     //QEA
#define QEB         PORTBbits.RB5     //QEB

// MACROS
#define VR2Value()      (ADCBUF0*100.0)/1023.0; //ADC en %
#define HallValue()     (PORTD & 0x0700) >> 8;


// TIPOS
typedef struct {
float 	speed,      //Velocidad del motor
		position, 	//Posición del motor
        dcycle;     //Valor duty-cycle
long    ecount;     //Contaje Encoder
char    running,    //Motor activo
        direction,	//Sentido de giro del motor
        fault;      //Driver en fallo
}MOTOR_INFO;		//Guarda Info del motor

void SetupPorts();                                                  //Configura puertos
void IniPB(void(*AttnFunction)(char PBnum, char PBsts));            //Inicializa pulsadores
void IniVR2();                                                      //Inicializa entrada analógica AN7 conectada a potenciometro VR2
void Wait(long count);                                              //Espera nº ciclos especificados
//LCD
void IniLCD();                                                      //Inicializa LCD (Driver HD44780)
char BusyLCD();                                                     //Estado ocupado/libre del LCD
void WriteCmdLCD(char cmd);                                         //Escribe un comando al registro de instrucciones del LCD
void WriteDataLCD(char data);                                       //Escribe un comando al registro de datos del LCD
void DeleteLCD();                                                   //Borra el LCD
void PosLCD(char col,char row);                                     //Posiciona curson LCD en columna, fila
void WriteTxtLCD(char *buffer, char col, char row);                 //Escribe texto en LCD
void WriteNumLCD(float number,int len,int dec,char col,char row);   //Escribe numero en LCD
//MOTOR
void IniHall(void(*AttnFunction)());                                //Inicializa sensor Hall
void IniPWM(void(*AttnFunction)());                                 //Inicializa control PWM
void OutMngr();                                                     //Gestiona la activacion de las salidas al motor
void StartPWM();                                                    //Arranca PWM
void StopPWM();                                                     //Para PWM
char RstDriver();                                                   //Reset Fallo Driver
void SetDC(float val);                                              //Escribe valor Duty-Cycle
//ENCODER
void IniQEI(void(*AttnIndxFunc)());                                 //Inicializa Encoder
void PosUpdate();                                                   //Actualiza posicion Absoluta
void RstQEI();                                                      //Reset posicion inicial Encoder
//AUXILIAR
void iniTimerBlink(void(*AttnFunction)());                        //Inicializa Timer para Intermitencia


void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void);    //Atención de irq de pulsadores
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void);   //Atención de irq de conversor A/D
void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void);   //Atención de irq de posicion Hall CAP1
void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void);   //Atención de irq de posicion Hall CAP2
void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt(void);   //Atención de irq de posicion Hall CAP3
void __attribute__((interrupt, no_auto_psv)) _FLTAInterrupt(void);  //Atención de irq Fallo Driver
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);   //Atención de irq Encoder
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void);   //Atención de irq T1 calc. velocidad
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt (void);   //Atención de irq T4 Intermitencia




