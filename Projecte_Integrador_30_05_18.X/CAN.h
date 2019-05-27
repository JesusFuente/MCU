#include <xc.h>

#define CAN1RXCID	C1RX0B1             //RX Datos CAN1 1º WORD CODE ID
#define CAN1RXDAT	C1RX0B2             //RX Buffer Datos CAN1
#define CAN1RXLEN	C1RX0DLCbits.DLC    //RX Longitud datos CAN1
#define CAN1RXSID   C1RX0SIDbits.SID    //RX ID CAN1


void IniCAN(void(*AttnFunction)());     //Inicializa Bus CAN1
float ReadCANFloat();                   //Lee dato tipo Float
int ReadCANInt();                       //Lee dato tipo Int
long ReadCANLong();                     //Lee dato tipo Long
char ReadCANChar();                     //Lee dato tipo Char
void WriteCANFloat(unsigned int SID,float value);
void WriteCANInt(unsigned int SID,int value);
void WriteCANLong(unsigned int SID,long value);
void WriteCANChar(unsigned int SID,char value);
void iniTimerCAN(void);                 //Inicializa timer envio datos CAN

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt (void);   //Atención de irq C1 CAN1
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void);   //Atención de irq T3 envio datos CAN