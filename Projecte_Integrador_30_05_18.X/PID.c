#include "pid.h"
#include <xc.h>
#include "dsPICDEM_MC1.h"

PID_DATA SpeedLoop;
PID_DATA PositLoop;

extern MOTOR_INFO Motor;

char LoopType;


void IniSpeedLoop()
{
    SpeedLoop.EUmax=4000.0;
    SpeedLoop.EUmin=0.0;
    SpeedLoop.MVmax=100.0; 
    SpeedLoop.MVmin=-100.0;
    
    SpeedLoop.KP=1.0;    //Proporcional
    SpeedLoop.KI=0.5;    //Integral (seg)
    SpeedLoop.KD=0.0;    //Derivativa (seg)
    
    SpeedLoop.Tupdt=SPEPIDTM;    
    SpeedLoop.action=PIDACTION_REV;
    
    SpeedLoop.mode=PIDMODE_MAN;
    SpeedLoop.MV=0.0;
    PIDSetSP(0.0,&SpeedLoop);
    LoopType=TYPE_SPEED;
    
    //INICIALIZA TIMER 2 PARA PID VELOCIDAD
    //Calculo de PR con prescalado a 64
    unsigned int PR=(SPEPIDTM*FCY)/64000.0;
    
    T2CON = 0X0020; // TCY interno prescalado 64
    TMR2 = 0;       // Reset timer    
    PR2 = PR;       
    _T2IF = 0;      // Borra Flag Interrupcion T2
    _T2IE = 1;      // Activa interrupciones T2
    T2CONbits.TON = 1;       // Activa T2
}

void IniPositLoop()
{
    PositLoop.EUmax=3700.0;
    PositLoop.EUmin=-3700.0;
    PositLoop.MVmax=8.0; 
    PositLoop.MVmin=-8.0;
    
    PositLoop.KP=2;    //Proporcional
    PositLoop.KI=1.0;   //Integral (seg)
    PositLoop.KD=0.05;    //Derivativa (seg)
    
    PositLoop.Tupdt=POSPIDTM;    
    PositLoop.action=PIDACTION_REV;
    
    PositLoop.mode=PIDMODE_MAN;
    PositLoop.MV=0.0;
    PIDSetSP(0.0,&PositLoop);
    LoopType=TYPE_POSIT;
    
    //INICIALIZA TIMER 2 PARA PID POSICION
    //Calculo de PR con prescalado a 64
    unsigned int PR=(POSPIDTM*FCY)/64000.0;
    
    T2CON = 0X0020; // TCY interno prescalado 64
    TMR2 = 0;       // Reset timer    
    PR2 = PR;       
    _T2IF = 0;      // Borra Flag Interrupcion T2
    _T2IE = 1;      // Activa interrupciones T2
    T2CONbits.TON = 1;       // Activa T2
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Establece valor de SP

void PIDSetSP(float SP,PID_DATA *PID)
{
    float SPScale;
    
    //Normalizamos SP en %
    SPScale = 100*(SP - PID->EUmin)/(PID->EUmax - PID->EUmin);
    //Acotamos límites
    if (SPScale<0.0)   SPScale=0.0;
    if (SPScale>100.0) SPScale=100.0;
    
    PID->SP=SPScale;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Lazo PID

float PIDLoop(float PV,PID_DATA *PID)
{
    float Tp,Ti,Td;
    float err,err_old;
    float PVScale,MV;
    char WINDUP=0;
    
    //Normalizamos PV en %
    PVScale=100*(PV - PID->EUmin)/(PID->EUmax - PID->EUmin);

    //Acotamos límites
    if (PVScale<0.0)   PVScale=0.0;
    if (PVScale>100.0) PVScale=100.0;
    
    if (PID->mode == PIDMODE_AUTO)
    {
        //Calculo error
        if (PID->action == PIDACTION_REV) //Accion Inversa
        {
            err=(PID->SP - PVScale);
            err_old=(PID->SP - PID->PV);
        }
        else    //Accion Directa
        {
            err=(PVScale - PID->SP);
            err_old=(PID->PV - PID->SP);
        }
    
        //Algoritmo PID
        Tp= err*PID->KP;
        Td=(1000.0 * (err-err_old) * PID->KD)/(PID->Tupdt);        
        //Anulacion Integral si es 0
        if (PID->KI > 0.0)  Ti=(PID->Acc_err * PID->Tupdt)/(1000.0 * PID->KI); 
        else Ti=0;
        
        MV = Tp+Ti+Td+ PID->Bias;
        
        //Control limites salida
        if (MV < PID->MVmin)
        {
            MV=PID->MVmin;
            if (err<0) WINDUP=1;
        }
        else if (MV > PID->MVmax)
        {
            MV=PID->MVmax;
            if (err>0) WINDUP=1;
        }
        PID->MV = MV;        
 
        //Control Antisaturacion
        if (!WINDUP)
        {
            PID->Acc_err += err;            
        }
        
    }
    else
    {
      PID->Acc_err=0;
      PID->Bias=PID->MV;
    }
    
    PID->PV=PVScale;
    return PID->MV;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Interrupcion Generadas T2
// Lazo PID velocidad

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void)
{
    if (LoopType)
    {
       SetDC(PIDLoop(Motor.position,&PositLoop));    
    }
    else
    {
       SetDC(PIDLoop(Motor.speed,&SpeedLoop));    
    } 
    _T2IF = 0;          //Borra Flag Interrupcion
}