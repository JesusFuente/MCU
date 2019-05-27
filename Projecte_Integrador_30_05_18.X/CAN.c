#include <xc.h>
#include "dsPICDEM_MC1.h"
#include "PID.h"


extern MOTOR_INFO Motor;
extern PID_DATA PositLoop;

void (*CANFn)();           //Función de atención a la irq del Bus CAN

void IniCAN(void(*AttnFunction)())
{
    CANFn=AttnFunction;    //Función de atención a la irq
    
    //INICIO REGISTROS CAN
    C1CTRLbits.CANCAP=0;    //Deshabilita módulo CAN
    C1CTRLbits.CSIDL=0;     //Continue CAN modile operation in IDLE mode
    C1CTRLbits.ABAT=0;      //No effect => No Abort pending transmissions bit
    C1CTRLbits.CANCKS=1;    //Fcan clock is Fcy
    
    C1CTRLbits.REQOP=4;             //Modo configuracion
    while(C1CTRLbits.OPMODE!=4);    //Espera modo configuracion
    
     //Configuració del BitRate => Bit Rate = 500 000 bps
    C1CFG1=0x0000;  // Tq = 2/Fcan = 2/(16 MHz) = 1/(8 000 000 Hz) = 125 ns
    C1CFG2=0x05B8;  // Phase segment buffer 1 lenght = 8*Tq and Phase segment buffer 2 lenght = 6*Tq => Total = 16*Tq
    
    // Mascaras del Buffer 0 de entrada
    C1RXM0SIDbits.SID=0;        // Mascara de aceptación, Acecta cualquier ID de recepcion
    C1RXM0SIDbits.MIDE=1;       // Tiene en cuenta EXIDE
    C1RXF0SIDbits.EXIDE=0;      // Acepta solo mensajes con ID estandar
    
    // Transmision de salida
    C1TX0CON = 3;  // misstge d'altra prioritat
    C1TX0EID = 0;  // ID Mensaje  estandard
    C1TX0SIDbits.TXIDE = 0;
    C1TX0SIDbits.SRR = 0;
    
    // Interrupciones
    IFS1bits.C1IF=0;            // Borra Flag Interrupcion
    IEC1bits.C1IE=1;            // Habilitació d'interrupció del CAN 1   
    C1INTF=0;                   // Borra Flag Interrupcion   
    C1INTE=0xFF;                // Habilitem les interrupcions
    
    //Arranque CAN
    C1CTRLbits.REQOP=0;         // Modo Normal
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Lee dato tipo Float del buffer de lectura CAN

float ReadCANFloat()
{
    float *pfloat;
    pfloat=&C1RX0B1;
    return *pfloat;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe dato tipo Float del buffer de lectura CAN

void WriteCANFloat(unsigned int SID,float value)
{
    float *pfloat;
    pfloat=&C1TX0B1;   
    
    *pfloat=value;
    C1TX0SIDbits.SID5_0= SID; 
    C1TX0SIDbits.SID10_6 = SID>>6; 
    C1TX0DLC=4<<3;         //Longitud de datos 4 bytes
    C1TX0CONbits.TXREQ=1;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Lee dato tipo Int del buffer de lectura CAN

int ReadCANInt()
{
    int *pint;    
    pint=&C1RX0B1;   
    return *pint;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe dato tipo int del buffer de lectura CAN

void WriteCANInt(unsigned int SID,int value)
{
    int *pint;
    pint=&C1TX0B1;   
    
    *pint=value;
    C1TX0SIDbits.SID5_0 = SID; 
    C1TX0SIDbits.SID10_6 = SID>>6; 
    C1TX0DLC=2<<3;         //Longitud de datos 2 bytes.
    C1TX0CONbits.TXREQ=1;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Lee dato tipo Char del buffer de lectura CAN

char ReadCANChar()
{
    char *pint;    
    pint=&C1RX0B1;   
    return *pint;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe dato tipo char del buffer de lectura CAN

void WriteCANChar(unsigned int SID,char value)
{
    int *pint;
    pint=&C1TX0B1;   
    
    *pint=value;
    C1TX0SIDbits.SID5_0 = SID; 
    C1TX0SIDbits.SID10_6 = SID>>6; 
    C1TX0DLC=1<<3;         //Longitud de datos 1 bytes.
    C1TX0CONbits.TXREQ=1;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Lee dato tipo long del buffer de lectura CAN

long ReadCANLong()
{
    long *plong;
    plong=&C1RX0B1;
    return *plong;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe dato tipo long del buffer de lectura CAN

void WriteCANLong(unsigned int SID,long value)
{
    long *plong;
    plong=&C1TX0B1;   
    
    *plong=value;
    C1TX0SIDbits.SID5_0= SID; 
    C1TX0SIDbits.SID10_6 = SID>>6; 
    C1TX0DLC=4<<3;         //Longitud de datos 4 bytes
    C1TX0CONbits.TXREQ=1;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generada por Bus CAN1

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt (void)
{ 
    if (C1INTFbits.TX0IF) C1INTFbits.TX0IF = 0;
    if (C1INTFbits.TX1IF) C1INTFbits.TX1IF = 0;
    if (C1INTFbits.TX2IF) C1INTFbits.TX2IF = 0;
    if (C1INTFbits.RX0IF)
    {
        C1INTFbits.RX0IF = 0;
        if (CANFn) CANFn();
        C1RX0CONbits.RXFUL = 0;
    }
    if (C1INTFbits.RX1IF)
    {
        C1INTFbits.RX1IF = 0;
        if (CANFn) CANFn();
        C1RX1CONbits.RXFUL = 0;
    }
    if (C1INTFbits.WAKIF) C1INTFbits.WAKIF = 0;
    if (C1INTFbits.ERRIF) C1INTFbits.ERRIF = 0;
    if (C1INTFbits.IVRIF) C1INTFbits.IVRIF = 0;
    
    C1INTF=0;         // Borra Flag Interrupcion
    IFS1bits.C1IF=0;  // Borra Flag Interrupcion
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa temporizador para envio datos a Matlab (cada 500 ms))

void iniTimerCAN(){
  //prescalado a 256
  //tiempo muestreo 500 ms;
    
  T3CON=0x8030;   // TCY interno prescalado 256
  TMR3=0;         // Reset timer
  PR3=31250;
  _T3IE = 1;
  _T3IF = 0;
  T3CONbits.TON = 1;       // Activa T3
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generada por temporizador T3 (500 ms)
// envia posicion actual al Matlab

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{
    char pos=(Motor.position)/POSPMAX;
    WriteCANChar(630,pos); //posicio
    _T3IF = 0;          //Borra Flag Interrupcion
}