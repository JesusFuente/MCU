#include <xc.h>
#include "dsPICDEM_MC1.h"
#include <stdio.h>

unsigned int PORTG_PRE = 0;     //Guarda estado previo puerto G (pulsadores)

unsigned int POSCNT_PRE = 0;    //Guarda cont previo encoder para calc veloc.
int Sp_Acc_Revol = 0;           //Guarda revoluc. acumuladas para calc veloc.
int Po_Acc_Revol = 0;           //Guarda revoluc. acumuladas para calc posic.

unsigned int StepTableDIR[]=    //Tabla de conmutacion giro directo
{0x0000,0x0210,0x2004,0x0204,0x0801,0x0810,0x2001,0x0000};
unsigned int StepTableREV[]=    //Tabla de conmutacion giro inverso
{0x0000,0x2001,0x0810,0x0801,0x0204,0x2004,0x0210,0x0000};

MOTOR_INFO Motor;

void (*HallFn)();                     //Función de atención a la irq del Hall
void (*PBFn)(char PBnum, char PBsts); //Función de atención a las irq de pulsadores
void (*DrvFailFn)();                  //Función de atención a la irq del Fallo Driver
void (*EncIndxFn)();                  //Función de atención a la irq del Index Encoder
void (*BlinkTmFn)();                  //Función de atención a la irq de Intermitencia

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Configura puertos

void SetupPorts(void)
{
// ============= Port A ==============

//        absent
//        absent
//        absent
//        absent

//        absent
//        absent
//        absent
//        absent

//        absent
// RA9    VREF-                 O   LED1 (Active high)  
// RA10   VREF+                 O   LED2 (Active high)  
//        absent

//        absent
//        absent
// RA14   INT3                  O   LED3 (Active high)   
// RA15   INT4                  O   LED4 (Active high)  

    LATA  = 0x0000;
    TRISA = 0x39FF;

// ============= Port B ==============

// RB0    PGD/EMUD/AN0/CN2      AI  Phase1 I /EMUD
// RB1    PGC/EMUC/AN1/CN3      AI  Phase2 I /EMUC
// RB2    AN2/SS1/LVDIN/CN4     AI  Phase3 I
// RB3    AN3/INDX/CN5          I   QEI Index

// RB4    AN4/QEA/CN6           I   QEI A
// RB5    AN5/QEB/CN7           I   QEI B
// RB6    AN6/OCFA              AI  PFC Hall  
// RB7    AN7                   AI  Pot (VR2)

// RB8    AN8                   AI  Bus Shunt
// RB9    AN9                   AI  VAC Sense
// RB10   AN10                  AI  Phase4 Shunt
// RB11   AN11                  AI  VBus Sense

// RB12   AN12                  AI  V Phase1
// RB13   AN13                  AI  V Phase2
// RB14   AN14                  AI  V Phase3
// RB15   AN15/OCFB/CN12        O   Fault'

    LATB  = 0x0000;
    TRISB = 0xFFFF;

// ============= Port C ==============

//        absent
// RC1    T2CK                  O   LCD R/W'
//        absent
// RC3    T4CK                  O   LCD RS 

//        absent
//        absent
//        absent
//        absent

//        absent
//        absent
//        absent
//        absent

//        absent
// RC13   EMUD1/SOSC2/CN1           EMUD1
// RC14   EMUC1/SOSC1/T1CK/CN0      EMUC1
// RC15   OSC2/CLKO

    LATC  = 0x0000;
    TRISC = 0xFFF5;

// ============= Port D ==============

// RD0    EMUC2/OC1             I/O LCD D0
// RD1    EMUD2/OC2             I/O LCD D1      
// RD2    OC3                   I/O LCD D2
// RD3    OC4                   I/O LCD D3

// RD4    OC5/CN13              O   Brake circuit firing line (Active high)
// RD5    OC6/CN14              O   PFC switch firing line (Active high)
// RD6 x    OC7/CN15  			O	//*** Output compare 7 for diagnostics	
// RD7 x  OC8/CN16/UPDN         O   LED FWD/REV

// RD8    IC1                   I   CAP1
// RD9    IC2                   I   CAP2
// RD10   IC3                   I   CAP3
// RD11 x IC4                   O   Fire Enable (PWM Output Enable) (Active low)

// RD12 x IC5
// RD13   IC6/CN19              O   LCD ENA
// RD14   IC7/CN20
// RD15   IC8/CN21

    LATD  = 0x0000;
    TRISD = 0xD70F;
    //TRISD = 0xDFFF;
// ============= Port E ==============

// RE0    PWM1L                 O   Phase1 L
// RE1    PWM1H                 O   Phase1 H
// RE2    PWM2L                 O   Phase2 L
// RE3    PWM2H                 O   Phase2 H

// RE4    PWM3L                 O   Phase3 L
// RE5    PWM3H                 O   Phase3 H
// RE6    PWM4L                 O   Phase4 L
// RE7    PWM4H                 O   Phase4 H

// RE8    FLTA/INT1             I   Fault'
// RE9    FLTB/INT2             O   Fault Reset (Active high)
//        absent
//        absent

//        absent
//        absent
//        absent
//        absent

    LATE  = 0x0000;
    TRISE = 0xFD00;

// ============= Port F ==============

// RF0    C1RX                  I   CAN Rx
// RF1    C1TX                  O   CAN Tx
// RF2    U1RX                  I   232 Rx
// RF3    U1TX                  O   232 Tx

// RF4    U2RX/CN17             I   485 Rx
// RF5    U2TX/CN18             O   485 Tx
// RF6    EMUC3/SCK1/INT0       I   ISO SCK1
// RF7    SDI1                  I   ISO SDI1

// RF8    EMUD3/SDO1            O   SDO1
//        absent
//        absent
//        absent

//        absent
//        absent
//        absent
//        absent

    LATF  = 0x0000;
    TRISF = 0xFED5;

// ============= Port G ==============

// RG0    C2RX                  O   485 RE'
// RG1    C2TX                  O   485 DE
// RG2    SCL                   I/O SCL
// RG3    SDA                   I/O SDA

//        absent
//        absent
// RG6    SCK2/CN8              I   Button 1 (S4) (Active low)
// RG7    SDI2/CN9              I   Button 2 (S5) (Active low)

// RG8    SDO2/CN10             I   Button 3 (S6) (Active low)
// RG9    SS2/CN11              I   Button 4 (S7) (Active low)
//        absent
//        absent

//        absent
//        absent
//        absent
//        absent

    LATG  = 0x0000;
    TRISG = 0xFFF0;

}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa interrupciones pulsadores

void IniPB(void(*AttnFunction)(char PBnum, char PBsts))
{
    PBFn=AttnFunction;
    
    _TRISG6 = 1;   //puerto PButton S4 como entrada
    _TRISG7 = 1;   //puerto PButton S5 como entrada
    _TRISG8 = 1;   //puerto PButton S6 como entrada
    _TRISG9 = 1;   //puerto PButton S7 como entrada
    
    _CN8PUE = 1;   //Desactiva pullups en CN8
    _CN8IE = 1;    //Activa interrupcion en PB_S4 (CN8)
    _CN9PUE = 1;   //Desactiva pullups en CN9
    _CN9IE = 1;    //Activa interrupcion en PB_S5 (CN9)
    _CN10PUE = 1;  //Desactiva pullups en CN10
    _CN10IE = 1;   //Activa interrupcion en PB_S6 (CN10)
    _CN11PUE = 1;  //Desactiva pullups en CN11
    _CN11IE = 1;   //Activa interrupcion en PB_S7 (CN11)
    
    PORTG_PRE = PORTG;  // Guarda estado anterior
    _CNIF = 0;          // Borra Flag Interrupcion
    _CNIP = 3;          // Establece interrupcion prioridad 3
    _CNIE = 1;          // Activa interrupciones CN
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generada por pulsadores

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt (void)
{
	unsigned int BitsChange;
    char PBnum=0, PBsts=0;
    
    BitsChange = PORTG ^ PORTG_PRE;    // Cambio de pulsadores    
    if (BitsChange & 0x0040)           // Pulsador S4
    {
        PBnum=4;
        if (!PBUTTON_S4) PBsts=1; 
        else PBsts=0; 
    }
    if (BitsChange & 0x0080)           // Pulsador S5
    {
        PBnum=5;
        if (!PBUTTON_S5) PBsts=1;
        else PBsts=0;
    }
    if (BitsChange & 0x0100)           // Pulsador S6
    {
        PBnum=6;
        if (!PBUTTON_S6) PBsts=1;
        else PBsts=0;
    }
    if (BitsChange & 0x0200)           // Pulsador S7
    {
        PBnum=7;
        if (!PBUTTON_S7) PBsts=1;
        else PBsts=0;
    }
    
    if (PBFn) PBFn(PBnum,PBsts);
    PORTG_PRE = PORTG;      // Guarda estado anterior
    _CNIF = 0;              // Borra Flag Interrupcion
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa entrada analógica AN7 conectada a potenciometro VR2

void IniVR2(void)
{
    ADPCFG |= 0xFF7F;   // RB7/AN7=Analog
    ADCON1 = 0x00E0;    // SSRC=111 Internal counter ends sampling and starts conversion
    ADCON2 = 0x0004;    // Interrupcion cada 2 muestras
    ADCON3 = 0x0F00;    // Sample time = 15Tad, Tad = internal Tcy/2
    ADCHS  = 0x0007;    // Conecta AN7 al CH0
    ADCSSL = 0;         // Inputs no scanned
    _ASAM = 1;          // Auto start sampling
    
    
    _ADON = 1;          // turn ADC ON
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Genera tiempo de Espera
// ciclos = (14 x count) + 24 , Tiempo=ciclos/FCY 

void Wait(long count)
{
    while(count--);
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Genera secuencia de arranque especifica para el driver HD44780
// Se configura para 4 bits de datos, dos lineas, 5x8 puntos/caracter

void IniLCD(void)
{
   Wait(Delay_20ms);    
   LATD = (LATD&0xFFF0)|0x0003; //Funcion set interface 8 bits
   TRISD = TRISD&0xFFF0;        //Puerto de datos como salida    
   LCDRW = 0;                   //R/!W escribir
   LCDENA = 1;                  //Alterna señal enable
   Wait(Delay_200us);
   LCDENA = 0;

   Wait(Delay_10ms);
   LATD = (LATD&0xFFF0)|0x0003; //Funcion set interface 8 bits 
   LCDENA = 1;                  //Alterna señal enable
   Wait(Delay_200us);
   LCDENA = 0;

   Wait(Delay_1ms);
   LATD = (LATD&0xFFF0)|0x0003; //Funcion set interface 8 bits
   LCDENA = 1;                  //Alterna señal enable
   Wait(Delay_200us);
   LCDENA = 0;
   
   Wait(Delay_1ms);
   LATD = (LATD&0xFFF0)|0x0002; //Funcion set interface 4 bits
   LCDENA = 1;                  //Alterna señal enable
   Wait(Delay_200us);
   LCDENA = 0;
    
                 
   WriteCmdLCD(0x28);   // Function set: 4 bits, 2 lineas, 5x8 puntos                               
   WriteCmdLCD(0x06);   // Entry Mode: Incr 1, NO shift
   WriteCmdLCD(0x0C);   // LCD ON, Cursor OFF, NO blink
   WriteCmdLCD(0x01);   // Borra LCD
   WriteCmdLCD(0x80);   // Cursor al inicio
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Retorna estado de ocupado del LCD

char BusyLCD(void)
{
    char fbusy;
    TRISD = TRISD|0x000F;   // Puerto de datos como entrada
    LCDRS = 0;
    LCDRW = 1;              // R/!W leer    
    Wait(Delay_200us);
    LCDENA = 1;
    Wait(Delay_200us);        // Espera datos del LCD
    
    if(LCDD3) {fbusy = 1;}  // Lee el flag de busy
    else      {fbusy = 0;}
    
    LCDENA = 0;
    Wait(Delay_200us);
    LCDRW = 0;              // R/!W escribir 
    return fbusy;
} 


//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe un comando al registro de instrucciones del LCD

void WriteCmdLCD(char cmd)
{
    char cmdTemp;
    cmdTemp = cmd;
    
    while(BusyLCD());       // Esperar lcd listo
	
    LCDRS = 0;              // Seleccion registro instruccion
    LCDRW = 0;              // R/!W escribir
    
    // Parte alta del byte
    LATD = (LATD & 0xFFF0) | (cmdTemp >> 4 & 0x000F); 
    TRISD = TRISD&0xFFF0;        //Puerto de datos como salida 
    LCDENA = 1;
    Wait(Delay_200us);
    LCDENA = 0;
    Wait(Delay_200us);
    
    // Parte baja del byte
    LATD = (LATD & 0xFFF0) | (cmd & 0x000F);
    LCDENA = 1;
    Wait(Delay_200us);
    LCDENA = 0;
    Wait(Delay_200us);
    
    TRISD = TRISD|0x000F;   // Puerto de datos como entrada
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe un comando al registro de datos del LCD

void WriteDataLCD(char data)
{
    char dataTemp=data;
   
    while(BusyLCD());       // Esperar lcd listo
	
    LCDRS = 1;              // Seleccion registro Datos
    LCDRW = 0;              // R/!W escribir
    
    // Parte alta del byte
    LATD = (LATD & 0xFFF0) | (dataTemp >> 4 & 0x000F); 
    TRISD = TRISD&0xFFF0;        //Puerto de datos como salida 
    LCDENA = 1;
    Wait(Delay_200us);
    LCDENA = 0;
    Wait(Delay_200us);
    
    // Parte baja del byte
    LATD = (LATD & 0xFFF0) | (data & 0x000F);
    LCDENA = 1;
    Wait(Delay_200us);
    LCDENA = 0;
    Wait(Delay_200us);
    
    TRISD = TRISD|0x000F;   // Puerto de datos como entrada
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Borra el LCD

void DeleteLCD(void)
{
    while(BusyLCD());    // Esperar lcd listo
    WriteCmdLCD(0x01);   // Borra LCD
    WriteCmdLCD(0x80);   // Cursor al inicio
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Posiciona curson en columna, fila
// col: posicion columna inicio
// row: posicion fila inicio

void PosLCD(char col, char row)
{
	WriteCmdLCD(0x80|col|(row << 6)); //Set DDRAM address
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe texto en LCD empezando por columna, fila
// buffer: cadena de caracteres
// col: posicion columna inicio
// row: posicion fila inicio

void WriteTxtLCD(char *buffer, char col, char row)
{
    PosLCD(col,row);
    while(*buffer!='\0')
    {
        WriteDataLCD(*buffer);
        buffer++;
    }
    Wait(Delay_200ms);
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Escribe numero en LCD con formato especificado
// number: numero a escribir
// len: longitud
// dec: numero decimales
// col: posicion columna inicio
// row: posicion fila inicio

void WriteNumLCD(float number,int len,int dec,char col,char row)
{
    char buffer[17];
    int i=0;
      
    sprintf(buffer, "%*.*f", len,dec,(double)number);    
    PosLCD(col,row);    
    while(buffer[i]!='\0')
    {
        WriteDataLCD(buffer[i]);
        i++;
    }
    Wait(Delay_200ms);
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa posicion Hall
// AttnFunction: funcion de atencion a la interrupcion (opcional)

void IniHall(void(*AttnFunction)())
{
    HallFn=AttnFunction;    //Función de atención a la irq
            
    IC1CON=1;       // Capture mode, every edge
    IC2CON=1;       // Capture mode, every edge
    IC3CON=1;       // Capture mode, every edge
    
    _TRISD8 = 1;    // Puerto CAP1 como entrada
    _TRISD9 = 1;    // Puerto CAP2 como entrada
    _TRISD10 = 1;   // Puerto CAP3 como entrada
    
    _IC1IF = 0;     // Borra Flag Interrupcion IC1
    _IC2IF = 0;     // Borra Flag Interrupcion IC2
    _IC3IF = 0;     // Borra Flag Interrupcion IC3
    
    _IC1IP = 4;     // Interrupcion prioridad 4 IC1
    _IC2IP = 4;     // Interrupcion prioridad 4 IC2
    _IC3IP = 4;     // Interrupcion prioridad 4 IC3
    
    _IC1IE = 1;     // Activa interrupciones IC1
    _IC2IE = 1;     // Activa interrupciones IC2
    _IC3IE = 1;     // Activa interrupciones IC3
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupciones Generadas posiciones Hall

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
    OutMngr();
    if (HallFn) HallFn();       //Si hay funcion configurada se ejecuta
    _IC1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)
{
    OutMngr();
    if (HallFn) HallFn();       //Si hay funcion configurada se ejecuta
    _IC2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt (void)
{
	OutMngr();
    if (HallFn) HallFn();       //Si hay funcion configurada se ejecuta
    _IC3IF = 0;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa control PWM con control de fallo FLTA por interrupcion
// AttnFunction: funcion de atencion a la interrupcion de fallo (opcional)

void IniPWM(void(*AttnFunction)())
{
    DrvFailFn=AttnFunction; //Función de atención a la irq
    
    PTCON = 0x0000;			// Tiempo base en "free running" sin escalados   
    OVDCON = 0x0000;		// Desactiva salidas
    PTPER = (FCY/FPWM)-1;   // Tiempo base del periodo
    SEVTCMP = PTPER;        // Tiempo base del evento especial (ADC)
    PWMCON1 = 0x0700;		// Salidas sin control PWM
    PWMCON2 = 0x0F00;		// Postscale x16 evento especial (ADC) (no usado)
    DTCON1 = 0x0000;        // No hace falta deadtime (cambios diagonales)
    DTCON2 = 0x0000;
    FLTACON = 0x0007;       // Posicion fallo todo desconectado
    
    _FLTAIP = 4;            // Interrupcion prioridad 4 FLTA
    _FLTAIF = 0;            // Borra Flag Interrupcion FLTA    
    
    _PTEN = 1;              // Arranca PWM
    
    SetDC(0);               // DutyC a 0%
    
    if (DRVFAIL)
    {
        Motor.fault=1;                  //Flag Motor en fallo
        if (DrvFailFn)  DrvFailFn();    //Si hay funcion configurada se ejecuta
    }
    else    _FLTAIE = 1;    // Activa interrupciones FLTA
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generadas fallo Driver FLTA

void __attribute__((interrupt, no_auto_psv)) _FLTAInterrupt (void)
{
    _FLTAIE = 0;            // Desactiva interrupciones FLTA
    PWMCON1 = 0x0700;		// Salidas desactivadas
    Motor.fault=1;          // Flag Motor en fallo
    Motor.running=0;        // Flag Motor parado
    if (DrvFailFn)  DrvFailFn();
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Gestiona la activacion de las salidas al motor

void OutMngr(void)
{
    int HV;      //Lee posicion actual
    HV=(int)(PORTD & 0x0700) >>8;
        
    if (!DRVFAIL)       //Si no está en fallo
    {
        //Activa las salidas apropiadas segun la posicion actual
        // y el sentido de giro
        if (Motor.direction)    OVDCON = StepTableREV[HV];
        else                    OVDCON = StepTableDIR[HV];
        
        PFC=1;
    }
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Arranca PWM

void StartPWM(void)
{
    if (!DRVFAIL)       //Si no está en fallo
    {
        OutMngr();   
        PWMCON1 = 0x0777;		// Salidas independientes habilitadas para PWM
        Motor.running=1;        // Flag Motor en marcha
    }
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Para PWM

void StopPWM(void)
{
    PWMCON1 = 0x0700;		// Salidas desactivadas
    Motor.running=0;        // Flag Motor parado
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Resetea fallo Driver motor
// Retorna 1 si se ha reseteado con exito

char RstDriver(void)
{
    DRVRST=1;
    //Wait(Delay_RST); 
    DRVRST=0;
    if (!DRVFAIL)
    {
        Motor.fault=0;  //Flag Motor en fallo
        _FLTAIF = 0;    //Borra Flag Interrupcion FLTA
        _FLTAIE = 1;    //Activa interrupciones FLTA
        return 1;
    }
    else return 0;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Establece valor de Duty-Cycle
// DC: Valor de Duty-Cycle en % (+ giro directo, - giro inverso)

void SetDC(float DC)
{
    if (DC>=0)
    {
        Motor.direction=0;
        if (DC>100)   DC=100;
    }
    else
    {
        Motor.direction=1;
        DC=-DC;
        if (DC>100)   DC=100;
    }
    Motor.dcycle=DC;
    PDC1 = (unsigned int)(DC*(FCY/FPWM)/50.0);
    PDC2 = PDC1;
	PDC3 = PDC1;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa encoder con filtro de entrada y temporizador T1 pra calc. veloc.
// AttnIndxFunc: funcion de atencion a la interrupcion de paso por INDX (opci)

void IniQEI(void(*AttnFunction)())
{
    EncIndxFn=AttnFunction; //Función de atención a la irq
    
	ADPCFG |= 0x0038; // Configura QEI pins como digital
    
    // QEICON - QEI Control Register
    // x4 mode con reset de contador por contaje max
    // Indicacion direccion en LED
   	QEICON = 0x0740;
    
	// DFLTCON - Digital Filter Control Register
    // Interrupcion error contaje activo
    // Pulso Minimo=30/(RPM*IPR)=30/(4000*1000)= 7.5us
    // Filtro = (FCY*Pulso Minimo)/3 => (14.75M*7.5u)/3=36.87
    // 36.87 -> 1:32 -> Se filtran pulsos<6.5uS
   	DFLTCON = 0x00EE;    
    
    POSCNT = 0;    // Inicializa Contaje
	MAXCNT = IPR;  // Contaje maximo

    _QEIIP = 4;    // Interrupcion prioridad 4
    _QEIIF = 0;    // Borra Flag Interrupcion QEI
    _QEIIE = 1;    // Activa interrupciones QEI
    
    //INICIALIZA TIMER 1 PARA CALCULO VELOCIDAD
    //Calculo de PR con prescalado a 8
    unsigned int PRSPE=(SPESTM*FCY)/8;
    
    T1CON = 0X0010; // TCY interno prescalado 8
    TMR1 = 0;       // Reset timer    
    PR1 = PRSPE;       
    _T1IF = 0;      // Borra Flag Interrupcion T1
    _T1IE = 1;      // Activa interrupciones T1
    T1CONbits.TON = 1;       // Activa T1
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generadas ENCODER (POR CNTERR O INDEX)

void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt (void)
{
    if (_UPDN)
        {
            Sp_Acc_Revol++;     //Incrementa revoluciones calc vel
            Po_Acc_Revol++;     //Incrementa revoluciones calc pos
        }
    else 
        {
            Sp_Acc_Revol--;     //Decrementa revoluciones calc vel
            Po_Acc_Revol--;     //Decrementa revoluciones calc pos
        }
        
    if (EncIndxFn)  EncIndxFn();
    
    _QEIIF = 0;    //Borra Flag Interrupcion QEI
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generadas T1
// Calcula velocidad instantanea RPM y la posicion en grad

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
    //Calculo velocidad en RPM
    int PosInc=POSCNT-POSCNT_PRE+(Sp_Acc_Revol*IPR);    
    Motor.speed=(60.0*PosInc)/(SPESTM*IPR);            
    POSCNT_PRE=POSCNT;
    Sp_Acc_Revol=0;
    
    PosUpdate();        //Actualiza Info posicion
    
    _T1IF = 0;          //Borra Flag Interrupcion
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Actualiza posicion absoluta Encoder

void PosUpdate(void)
{
    float count;
    //Calculo posicion en grados
    Motor.ecount=POSCNT+(Po_Acc_Revol*IPR);
    count=(float)Motor.ecount;
    Motor.position=(360.0*count)/IPR; 
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Resetea posicion inicial Encoder

void RstQEI(void)
{
    POSCNT = 0;
    Po_Acc_Revol=0;
    Motor.ecount=0;
    Motor.position=0;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa Timer 4 para intermitencia (500 ms)

void iniTimerBlink(void(*AttnFunction)()){
  //prescalado a 256
  //tiempo muestreo 500 ms;
  
  BlinkTmFn=AttnFunction; //Función de atención a la irq
    
  T4CON=0x8030;   // TCY interno prescalado 256
  TMR4=0;         // Reset timer
  PR4=31250;
  _T4IE = 1;
  _T4IF = 0;
  T4CONbits.TON = 1;       // Activa T4
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generada por temporizador T4 (500 ms)

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt (void)
{
    if (BlinkTmFn)  BlinkTmFn();
    _T4IF = 0;          //Borra Flag Interrupcion
}


