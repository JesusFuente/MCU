
#define PIDMODE_MAN      0    //normal auto mode
#define PIDMODE_AUTO     1    //normal auto mode

#define PIDACTION_REV    0    //Accion Inversa
#define PIDACTION_DIR    1    //Accion Directa

#define TYPE_SPEED    0    //Control lazo velocidad
#define TYPE_POSIT    1    //Control lazo posición

#define SPEPIDTM        10    //Tiempo muestreo PID Veloc (ms)
#define POSPIDTM        10    //Tiempo muestreo PID Veloc (ms)

#define POSPMAX        36.0    //Escalado posición max 10 vueltas grados/%



typedef struct
{
    float SP;    //Setpoint %
    float PV;    //Process Value %
    float MV;    //Manipulated Value %
    
    float KP;    //Proporcional
    float KI;    //Integral (seg)
    float KD;    //Derivativa (seg)
    
    float EUmin; //Unidades Ingen. min para PV y SP
    float EUmax; //Unidades Ingen. max para PV y SP
    float MVmin; //MV min en %
    float MVmax; //MV max en %
    
    float Tupdt;           //Periodo actualizacion lazo (ms)
    float Acc_err;         //Error Acumulado
    float Bias;            //MV inicial
    unsigned char action;  //PID Accion
    unsigned char mode;    //PID Modo
} PID_DATA;    

void IniSpeedLoop();
void IniPositLoop();
void PIDSetSP(float SP,PID_DATA *PID);
float PIDLoop(float PV,PID_DATA *PID);

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void);   //Atención de irq T2 PID
