/**********************************************************************
* � 2005 Microchip Technology Inc.
*
* FileName:        main.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx
* Compiler:        MPLAB� C30 v3.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author          	Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Vinaya Skanda 	10/18/06  First release of source file
* Vinaya Skanda		07/25/07  Updates from Joe Supinsky and Jatinder Gharoo incorporated
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
* This code is tested on Explorer-16 board with ECAN PICTail Card.
* The device used is dsPIC33FJ256GP710 controller 
*
* The Processor starts with the External Crystal without PLL enabled and then the Clock is switched to PLL Mode.
*************************************************************************************************/

#if defined(__dsPIC33F__)
#include "p33fxxxx.h"
#elif defined(__PIC24H__)
#include "p24hxxxx.h"
#endif

#include "ECAN1Config.h"
#include "ECAN2Config.h"
#include "common.h"
#include "TIMER.h"

//  Macros for Configuration Fuse Registers 
_FOSCSEL(FNOSC_FRC); 
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_XT);  
								// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
								// OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystanl


_FWDT(FWDTEN_OFF);              // Watchdog Timer Enabled/disabled by user software
								// (LPRC can be disabled by clearing SWDTEN bit in RCON register


// Define ECAN Message Buffers
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));
ECAN2MSGBUF ecan2msgBuf __attribute__((space(dma),aligned(ECAN2_MSG_BUF_LENGTH*16)));

// CAN Messages in RAM
mID rx_ecan1message;
mID rx_ecan2message;

// Prototype Declaration
void oscConfig(void);
void clearIntrflags(void);
void ecan1WriteMessage(void);
void ecan2WriteMessage(void);
void LED_config();
void Blink();

int time=0;



int main(void)
{

/* Configure Oscillator Clock Source 	*/
	oscConfig();

/* Clear Interrupt Flags 				*/
	clearIntrflags();
    
/* LEDs configuration                   */
    LED_config();
    
/* Timer Interrupt configuration        */
/* Initialize Timer 1 for 32KHz real-time clock operation */
    Init_Timer1();

/* ECAN1 Initialisation 		
   Configure DMA Channel 0 for ECAN1 Transmit
   Configure DMA Channel 2 for ECAN1 Receive */
	ecan1Init();
	dma0init();	
	dma2init();

/* Enable ECAN1 Interrupt */ 				
    	
	IEC2bits.C1IE = 1;
	C1INTEbits.TBIE = 1;	
	C1INTEbits.RBIE = 1;

/* ECAN2 Initialisation 		
   Configure DMA Channel 1 for ECAN2 Transmit
   Configure DMA Channel 3 for ECAN2 Receive */
	ecan2Init();
	dma1init();	
	dma3init();

/* Enable ECAN2 Interrupt */ 
	
	IEC3bits.C2IE = 1;
	C2INTEbits.TBIE = 1;	
	C2INTEbits.RBIE = 1;

 
/* Write a Message in ECAN1 Transmit Buffer	
   Request Message Transmission			*/
	ecan1WriteMessage();
	C1TR01CONbits.TXREQ0=1;	
	


/* Write a Message in ECAN2 Transmit Buffer
   Request Message Transmission			*/
	ecan2WriteMessage();
	C2TR01CONbits.TXREQ0=1;
	

/* Loop infinitely */

	while (1)
    {
        //ecan1WriteMessage();
        //C1TR01CONbits.TXREQ0=1;	
    }
	
}




/* ECAN1 buffer loaded with Identifiers and Data */
void ecan1WriteMessage(void){

/* Writing the message for Transmission
ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each

*/

	ecan1WriteTxMsgBufId(0,0x1FFEFFFF,1,0);
	ecan1WriteTxMsgBufData(0,8,0x1111,0x2222,0x3333,0x4444);

}

/* ECAN2 buffer loaded with Identifiers and Data */

void ecan2WriteMessage(void){

/* Writing the message for Transmission

ecan2WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
ecan2WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer Number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission


dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each


*/

ecan2WriteTxMsgBufId(0,0x1FFEFFFF,1,0);
ecan2WriteTxMsgBufData(0,8,0xaaaa,0xbbbb,0xcccc,0xdddd);

}



/******************************************************************************
*                                                                             
*    Function:			rxECAN1
*    Description:       moves the message from the DMA memory to RAM
*                                                                             
*    Arguments:			*message: a pointer to the message structure in RAM 
*						that will store the message. 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void rxECAN1(mID *message)
{
	unsigned int ide=0;
	unsigned int srr=0;
	unsigned long id=0,d;
			
	/*
	Standard Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits
	
	Substitute Remote Request Bit
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	*/
	/* read word 0 to see the message type */
	ide=ecan1msgBuf[message->buffer][0] & 0x0001;	
	srr=ecan1msgBuf[message->buffer][0] & 0x0002;	
	
	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan1msgBuf[message->buffer][0] & 0x1FFC) >> 2;		
		message->frame_type=CAN_FRAME_STD;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1msgBuf[message->buffer][0] & 0x1FFC;		
		message->id=id << 16;
		id=ecan1msgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan1msgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;		
		message->frame_type=CAN_FRAME_EXT;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(srr==1)
	{
		message->message_type=CAN_MSG_RTR;	
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan1msgBuf[message->buffer][3];
		message->data[1]=(unsigned char)((ecan1msgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan1msgBuf[message->buffer][4];
		message->data[3]=(unsigned char)((ecan1msgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan1msgBuf[message->buffer][5];
		message->data[5]=(unsigned char)((ecan1msgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan1msgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan1msgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=(unsigned char)(ecan1msgBuf[message->buffer][2] & 0x000F);
	}	
}


/******************************************************************************
*                                                                             
*    Function:			rxECAN2
*    Description:       moves the message from the DMA memory to RAM
*                                                                             
*    Arguments:			*message: a pointer to the message structure in RAM 
*						that will store the message. 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void rxECAN2(mID *message)
{
	unsigned int ide=0;
	unsigned int srr=0;
	unsigned long id=0,a1;
			
	/*
	Standard Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits
	
	Substitute Remote Request Bit
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	*/
	/* read word 0 to see the message type */
	ide=ecan2msgBuf[message->buffer][0] & 0x0001;	
	srr=ecan2msgBuf[message->buffer][0] & 0x0002;	
	
	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan2msgBuf[message->buffer][0] & 0x1FFC) >> 2;		
		message->frame_type=CAN_FRAME_STD;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan2msgBuf[message->buffer][0] & 0x1FFC;		
		message->id=id << 16;
		id=ecan2msgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan2msgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;		
		message->frame_type=CAN_FRAME_EXT;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(srr==1)
	{
		message->message_type=CAN_MSG_RTR;	
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan2msgBuf[message->buffer][3];
		message->data[1]=(unsigned char)((ecan2msgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan2msgBuf[message->buffer][4];
		message->data[3]=(unsigned char)((ecan2msgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan2msgBuf[message->buffer][5];
		message->data[5]=(unsigned char)((ecan2msgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan2msgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan2msgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=(unsigned char)(ecan2msgBuf[message->buffer][2] & 0x000F);
	}	
}





void clearIntrflags(void){
/* Clear Interrupt Flags */

	IFS0=0;
	IFS1=0;
	IFS2=0;
	IFS3=0;
	IFS4=0;
}


void oscConfig(void){

/*  Configure Oscillator to operate the device at 40Mhz
 	Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
 	Fosc= 8M*40/(2*2)=80Mhz for 8M input clock */

	PLLFBD=38;					/* M=40 */
    //PLLFBD=14;					/* M=16 */
    //PLLFBD=30;					/* M=32 */
	CLKDIVbits.PLLPOST=0;		/* N1=2 */
    //CLKDIVbits.PLLPOST=1;		/* N1=2 */
	CLKDIVbits.PLLPRE=0;		/* N2=2 */
	OSCTUN=0;					/* Tune FRC oscillator, if FRC is used */

/* Disable Watch Dog Timer */

	RCONbits.SWDTEN=0;

/* Clock switch to incorporate PLL*/
	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
													// Oscillator with PLL (NOSC=0b011)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur	

    
    
/* Wait for PLL to lock */

	while(OSCCONbits.LOCK!=1) {};
}


void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)  
{    
	IFS2bits.C1IF = 0;        // clear interrupt flag
	if(C1INTFbits.TBIF)
    { 
    	C1INTFbits.TBIF = 0;
    } 
 
    if(C1INTFbits.RBIF)
    {      
		// read the message 
	    if(C1RXFUL1bits.RXFUL1==1)
	    {
	    	rx_ecan1message.buffer=1;
	    	C1RXFUL1bits.RXFUL1=0;
	    }	    
	    rxECAN1(&rx_ecan1message);
        
        if(rx_ecan1message.id == 500)
        {
            switch((int)rx_ecan1message.data[0])
            {
                case 0:  //Luces apagadas
                {
                    LED_D6=0;
                    asm("NOP");
                    LED_D7=0;
                    asm("NOP");
                    break;
                }
                case 1:  //Luces cortas
                {
                    LED_D6=1;
                    asm("NOP");
                    LED_D7=0;
                    asm("NOP");
                    break;
                }
                case 3:  //Luces largas
                {
                    LED_D6=1;
                    asm("NOP");
                    LED_D7=1;
                    asm("NOP");
                    break;
                }
            }
        }
        else
        {
            if(rx_ecan1message.id == 550)
            {
                blinking = (int)rx_ecan1message.data[0];
            }
        }
        
        
		C1INTFbits.RBIF = 0;
	}
}


void __attribute__((interrupt, no_auto_psv))_C2Interrupt(void)  
{
	IFS3bits.C2IF = 0;        // clear interrupt flag
	if(C2INTFbits.TBIF)
    { 
		C2INTFbits.TBIF = 0;
    } 
    
    if(C2INTFbits.RBIF)
     {      
		// read the message 
	    if(C2RXFUL1bits.RXFUL1==1)
	    {
	    	rx_ecan2message.buffer=1;
	    	C2RXFUL1bits.RXFUL1=0;
	    }	    
	    rxECAN2(&rx_ecan2message); 	    	    
		C2INTFbits.RBIF = 0;
     }
}
 

//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
   IFS0bits.DMA0IF = 0;          // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
   IFS0bits.DMA1IF = 0;          // Clear the DMA1 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
   IFS1bits.DMA2IF = 0;          // Clear the DMA2 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
{
   IFS2bits.DMA3IF = 0;          // Clear the DMA3 Interrupt Flag;
}

void LED_config(void)
{
    _TRISA0 = 0;
    _TRISA1 = 0;
    _TRISA2 = 0;
    _TRISA3 = 0;
    _TRISA4 = 0;
}

void Blink()
{
    switch(blinking){
        case 0:  //OFF
            {
                LED_D8=0;
                asm("NOP");
                break;
            }
        case 1:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                    asm("NOP");
                }
                else
                {
                    LED_D8=0;
                    asm("NOP");
                }
                break;
            }
        case 2:  //Right
            {
                LED_D8=0;
                asm("NOP");
                break;
            }
        case 3:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                    asm("NOP");
                }
                else
                {
                    LED_D8=0;
                    asm("NOP");
                }
                break;
            }
        case 4:  //Left
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                    asm("NOP");
                }
                else
                {
                    LED_D8=0;
                    asm("NOP");
                }
                break;
            }
        case 5:  //Both
            {
                if (LED_D8==0)
                {
                    LED_D8=1;
                    asm("NOP");
                }
                else
                {
                    LED_D8=0;
                    asm("NOP");
                }
                break;
            }
    }
}
/*
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Inicializa Timer 4 para intermitencia (500 ms)

void iniTimerBlink()
{
  //prescalado a 256
  //tiempo muestreo 500 ms;
    
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
    Blink();
    _T4IF = 0;          //Borra Flag Interrupcion
}
*/

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt( void )
{
	
	if(time >= 630)
    {
        //LED_D10 = (LED_D10 ^ 0x1);
        Blink();
        time = 0;
    }
    
    time++;
	/* reset Timer 1 interrupt flag */
 	IFS0bits.T1IF = 0;
}
