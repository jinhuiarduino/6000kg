#include "c8051F310.h"                                 
#include <stdio.h>
#include <intrins.h>		//void _nop_ () ,unsigned long _lrol_ (unsigned long val,unsigned char n)

//-----------------------------------------------------------------------------
// User-defined types, structures, unions etc
//-----------------------------------------------------------------------------
#ifndef BYTE
#define BYTE unsigned char
#endif

#ifndef UINT
#define UINT unsigned int
#endif


#define  F_SCK_MAX         3000000     // Max SCK freq (Hz)
#define  flash_CAPACITY    0x400000

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F31x
//-----------------------------------------------------------------------------

sfr16 ADC0     = 0xbd;                 // ADC0 result
sfr16 TMR2     = 0xCC;                 // Timer2 low and high bytes together
sfr16 TMR2RL   = 0xCA;    					// TIMER 2 RELOAD low and high bytes together

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK       12250000          // SYSCLK frequency in Hz
#define BAUDRATE     115200              // Baud rate of UART in bps

#define WriteEn  	 0x06
#define WriteDi		 0x04
#define ReadID       0x9f
#define ReadSR       0x05
#define WriteSR      0x01
#define Read         0x03
#define fastRead     0x0b
#define PP			 0x02
#define SE			 0xd8
#define BE    		 0xc7
//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);
void PORT_Init (void);
void ADC0_Init(void);
void UART0_Init (void);
void SPI0_Init(void);
void flashWriteAddress (unsigned long address);
void flashRead (unsigned long address);
void flashErase(void);
void flashWEn(void);
void flashWDi(void);
void flashErase(void);
void flashWriteSR(void);
void delay(void);
void SSTFlash_Init (void);
void Timer2_Init (void);
void Ext_Interrupt_Init (void);
void flashSlectReadP10 (unsigned long address);
BYTE flashSlectReadAA (unsigned long address);
void flashSlectReadP2 (unsigned long address);
void Delay_ms (BYTE time_ms);
void TIMER2_Init (void);
unsigned char Read_MEM (unsigned long address);

sbit P10 = P1^0;
sbit P11 = P1^1;
sbit P12 = P1^2;
sbit P13 = P1^3;

sbit P20 = P2^0;
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
BYTE Flag = 0;
unsigned long address1 = 0,address2= 0,address3= 0,address = 0x000000;

void main (void) 
{
//	BYTE i;
    PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer 

    SYSCLK_Init ();                     // Initialize system clock to 
                                        // 24.5MHz/2
    PORT_Init ();                       // Initialize crossbar and GPIO
                                        // overflows to trigger ADC
    UART0_Init();                       // Initialize UART0 for printf's
//   ADC0_Init();                       // Initialize ADC0
	SPI0_Init();
	SSTFlash_Init (); 
	Timer2_Init();
	Ext_Interrupt_Init();
//    TIMER2_Init ();
 
//flashErase();	while(1);
//flashRead(address);
//-------------------------------------------------------------------------------------------------------
 

	if((P10 == 1)&&(P11 == 0)&&(P12 == 0)&&(P13 == 0))               //P1 0001  for P2
	{
	Delay_ms(500);
	flashRead(0x00);
	while(1);
	}
	if((P10 == 0)&&(P11 == 0)&&(P12 == 0)&&(P13 == 1))				//P1 1000 for P2
	{
	Delay_ms(500);
	flashRead(0x00);
	while(1);
	}

	if((P10 == 0)&&(P11 == 1)&&(P12 == 1)&&(P13 == 0))			//P1 0110 for erase
	{
	Delay_ms(500);
  	flashErase();
		while(1);
	}
	Delay_ms(300);
 	if(Read_MEM (0x3ffff0) == 0xaa)
	{
	Delay_ms(5);
	 if(Read_MEM (0x3ffff0) == 0xaa)
	  {

	  while(1);
	  }
	}  
//
	while(!TI0)
	{
	SBUF0 = Read_MEM (0x3ffff0); 
	}
	TI0 = 0; 

	while(!TI0)
	{
	SBUF0 = Read_MEM (0x3ffff0);
	}
	TI0 = 0; 

//	
	EA = 1;							         // enable global interrupts

	
	while(1)
	{

	}
	  

}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// This routine initializes the system clock to use the internal 12MHz 
// oscillator as its clock source.  Also enables missing clock detector reset.
//
//-----------------------------------------------------------------------------
void SYSCLK_Init (void)
{

    OSCICN    = 0x82;
    RSTSRC    = 0x04;                      // enable missing clock detector

}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
    // P0.0  -  SCK  (SPI0), Push-Pull,  Digital
    // P0.1  -  MISO (SPI0), Open-Drain, Digital
    // P0.2  -  MOSI (SPI0), Push-Pull,  Digital
    // P0.3  -  NSS  (SPI0), Push-Pull,  Digital
    // P0.4  -  TX0 (UART0), Push-Pull,  Digital
    // P0.5  -  RX0 (UART0), Open-Drain, Digital
    // P0.6  -  Unassigned,  Open-Drain, Digital
    // P0.7  -  Unassigned,  Open-Drain, Digital


//-----------------------------------------------------------------------------
void PORT_Init (void)
{

	P0SKIP    = 0x80;
    P0MDOUT   = 0x15;
    XBR0      = 0x03;
    XBR1      = 0x40;
}



//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configures ADC0 to make single-ended analog measurements on pin P1.1
//  
//-----------------------------------------------------------------------------

void ADC0_Init (void)
{

   ADC0CN = 0x02;                      // ADC0 disabled, normal tracking, 
                                       // conversion triggered on TMR2 overflow

   REF0CN = 0x0B;                      // Enable on-chip VREF and buffer

   AMX0P  = 0x01;                       // ADC0 positive input = P1.1
   AMX0N  = 0x1F;                       // ADC0 negative input = GND
                                       // i.e., single ended mode

   ADC0CF = ((SYSCLK/3000000)-1)<<3;   // set SAR clock to 3MHz

   ADC0CF |= 0x04;                     // left-justify results 

   EIE1 |= 0x08;                       // enable ADC0 conversion complete int.

   AD0EN = 1;                          // enable ADC0
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//
//-----------------------------------------------------------------------------

void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON |=  0x08;                  // T1M = 1; SCA1:0 = xx
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else if (SYSCLK/BAUDRATE/2/256 < 48) {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   } else {
      while (1);                       // Error.  Unsupported baud rate
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   TI0 = 1;                            // Indicate TX0 ready
}


//-----------------------------------------------------------------------------
// SPI0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configures SPI0 to use 4-wire Single-Master mode. The SPI timing is 
// configured for Mode 0,0 (data centered on first edge of clock phase and 
// SCK line low in idle state). The SPI clock is set to 2 MHz. The NSS pin 
// is set to 1.
//
//-----------------------------------------------------------------------------
void SPI0_Init()
{
  SPI0CFG   = 0x40;
  SPI0CN    = 0x0D;
   
   // The equation for SPI0CKR is (SYSCLK/(2*F_SCK_MAX))-1
   SPI0CKR   = (SYSCLK/(2*F_SCK_MAX)) - 1;

}

//------------------------------------------------------------------------------
//delay time 
//------------------------------------------------------------------------------
void delay()
{
	UINT i,j;
	 for(i= 5000;i>0;i--)
	  for(j=1;j>0;j--)
	   {_nop_ ();}

}

//all for M25P32 flash :write, read,erase ...

//---------------------------------------------------------------------------------
void flashWEn()
{
   NSSMD0   = 0;                       // Activate Slave Select
   SPI0DAT  = WriteEn;
   while (!SPIF);
   SPIF     = 0;
   NSSMD0   = 1;                       // Deactivate Slave Select
   	
}
//------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
void flashWDi()
{
   NSSMD0   = 0;                       // Activate Slave Select
   SPI0DAT  = WriteDi;
   while (!SPIF);
   SPIF     = 0;
   NSSMD0   = 1;                       // Deactivate Slave Select
}

//------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
void flashWriteAddress (unsigned long address)
{

   SPI0DAT  = (BYTE)((address>>16) & 0x000000FF);
   while (!SPIF);
   SPIF     = 0;
   SPI0DAT  = (BYTE)((address>>8) & 0x000000FF);

   while (!SPIF);
   SPIF     = 0;
   SPI0DAT  = (BYTE)(address & 0x000000FF);

   while (!SPIF);
   SPIF     = 0;

}
//--------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void flashRead (unsigned long address)
{


	   while(address < flash_CAPACITY)
	{
	   	
	      while(!TI0)
		{
		SBUF0 = Read_MEM (address);
		}
		TI0 = 0; 
		address++;		

	}
	while(1);


   
}
//-------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
void flashErase()
{
	flashWEn();
	_nop_ ();
   NSSMD0   = 0;                       // Activate Slave Select
   SPI0DAT  = BE;
   while (!SPIF);
   SPIF     = 0;
   NSSMD0   = 1;                       // Deactivate Slave Select


}
//-----------------------------------------------------------------------------
void flashWriteSR()
{
	flashWEn();

   NSSMD0   = 0;                       // Activate Slave Select
   SPI0DAT  = WriteSR;
   while (!SPIF);
   SPIF     = 0;

    SPI0DAT  = 0;
   while (!SPIF);
   SPIF     = 0;
   NSSMD0   = 1;                       // Deactivate Slave Select

}

/////////////////////////////////////////////////////////////////////////////
void SSTFlash_Init (void)
{
   NSSMD0 = 0;                         // enable the flash

   // send the enable write status register command
   SPI0DAT = WriteEn;                     // load the XMIT register
   while (TXBMT != 1)                  // wait until EWSR command is moved into
   {                                   // the XMIT buffer
   }
   SPIF = 0;
   while (SPIF != 1)                   // wait until the SPI finishes sending
   {                                   // the EWSR command to the flash
   }
   SPIF = 0;

   NSSMD0 = 1;                         // allow the command to execute

   NSSMD0 = 0;                         // enable the flash

   // send the write status register command and clear the BP bits
   SPI0DAT = WriteSR;                     // load the XMIT register
   while (TXBMT != 1)                  // wait until the XMIT register can
   {                                   // accept more data
   }
   SPI0DAT = 0x00;                     // set the block protection bits to 0
   while (TXBMT != 1)                  // wait until the data is moved into
   {                                   // the XMIT buffer
   }
   SPIF = 0;
   while (SPIF != 1)                   // wait until the SPI finishes sending
   {                                   // the data to the flash
   }
   SPIF = 0;

   NSSMD0 = 1;                         // allow the command to execute
}
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

void Ext_Interrupt_Init (void)
{

   TCON |= 0x01;                        // /INT 0   edge triggered

   IT01CF = 0x0F;                      // /INT0 active hihg /INT0 on P0.7

   EX0 = 1;                            // Enable /INT0 interrupts
}

//-----------------------------------------------------------------------------
// /INT0 ISR
//-----------------------------------------------------------------------------
//
// Whenever a negative edge appears on P0.7
// The interrupt pending flag is automatically cleared by vectoring to the ISR
//
//-----------------------------------------------------------------------------
void INT0_ISR (void) interrupt 0
{
	if(P20)
	{
	  Delay_ms(5);
	  if(P20)
	  {
	    Flag = 1;
	   flashWEn();							

	   NSSMD0   = 0;
                       
	   SPI0DAT  = PP;
	   while (!SPIF);
	   SPIF     = 0;

	   SPI0DAT  = (BYTE)((0x3ffff0>>16) & 0x000000FF);
	   while (!SPIF);
	   SPIF     = 0;
	   SPI0DAT  = (BYTE)((0x3ffff0>>8) & 0x000000FF);
	   while (!SPIF);
	   SPIF     = 0;
	   SPI0DAT  = (BYTE)(0x3ffff0 & 0x000000FF);
	   while (!SPIF);
	   SPIF     = 0;
		SPI0DAT  = 0xaa;	
	    while (!SPIF);
	    SPIF  = 0;
	
	    NSSMD0   = 1;
		Delay_ms(5);

	  }
	}
}
//---------------------------------------------------------------------------------
void Timer2_Init (void)
{
   TMR2CN = 0x00;                      // Stop Timer2; Clear TF2;
                                       // use SYSCLK as timebase, 16-bit
                                       // auto-reload
   CKCON |= 0x10;                      // Select SYSCLK for timer 2 source
   TMR2RL = 65535 - (SYSCLK / 10000);  // Init reload value for 10 us
   TMR2 = 0xffff;                      // Set to reload immediately
   ET2 = 1;                            // Enable Timer2 interrupts
   TR2 = 1;  
}
//-------------------------------------------------------------------------------
//void ADC0_ISR (void) interrupt 10

void Timer2_ISR (void) interrupt 5
{

   TF2H = 0; 
		
 	if(address < flash_CAPACITY)
    {

		if(Flag)
		{
	   flashWEn();							

	   NSSMD0   = 0;
                       
	   SPI0DAT  = PP;
	   while (!SPIF);
	   SPIF     = 0;

	   SPI0DAT  = address1;
	   while (!SPIF);
	   SPIF     = 0;
	   SPI0DAT  = address2;
	   while (!SPIF);
	   SPIF     = 0;
	   SPI0DAT  = address3;
	   while (!SPIF);
	   SPIF     = 0;
		SPI0DAT  = P2;	
	    while (!SPIF);
	    SPIF  = 0;
	
	    NSSMD0   = 1;
		address = address+1;		
		address1 = (BYTE)((address>>16) & 0x000000FF);
		address2 = (BYTE)((address>>8) & 0x000000FF);
		address3 = (BYTE)(address & 0x000000FF);		
		}
/*
if(Flag)
{
	while(!TI0)
	{
	SBUF0 = P2;
	}
	TI0 = 0; 

}
*/
    }


}
//-----------------------------------------------------------------------------

/*-------------------------------------------------------------------------------------

void TIMER2_Init (void)
{
   CKCON    |= 0x10;
}
///////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
void Delay_ms (BYTE time_ms)
{
   TR2   = 0;                          // Stop timer
   TF2H  = 0;                          // Clear timer overflow flag
   TMR2  = -( (UINT)(SYSCLK/1000) * (UINT)(time_ms) );
   TR2   = 1;                          // Start timer
   while (!TF2H);                      // Wait till timer overflow occurs
   TR2   = 0;                          // Stop timer
}
*/
void Delay_ms (BYTE time_ms)
{
	int i,j;
	for(i=1000;i>0;i--)
		{
		 for(j=time_ms;j>0;j--)
		  {;}
		}
}

unsigned char Read_MEM (unsigned long address)
{
   

   NSSMD0 = 0;                         // enable the flash

   // send the read instruction
   SPI0DAT = Read;                     // load the XMIT register
   while (TXBMT != 1)                  // wait until the command is moved into
   {                                   // the XMIT buffer
   }
   SPI0DAT = (BYTE)((address>>16) & 0x000000FF);       // load the high byte of the address
   while (TXBMT != 1)                  // wait until the data is moved into
   {                                   // the XMIT buffer
   }
   SPI0DAT = (BYTE)((address>>8) & 0x000000FF);       // load the middle byte of the address
   while (TXBMT != 1)                  // wait until the data is moved into
   {                                   // the XMIT buffer
   }
   SPI0DAT = (BYTE)(address & 0x000000FF);     // load the low byte of the address
   while (TXBMT != 1)                  // wait until the data is moved into
   {                                   // the XMIT buffer
   }
   SPI0DAT = 0xFF;                     // load junk data in order to receive
                                       // data from the flash
   while (TXBMT != 1)                  // wait until the junk data is moved
   {                                   // into the XMIT buffer
   }
   SPIF = 0;
   while (SPIF != 1)                   // wait until the read data is received
   {
   }
   SPIF = 0;

   NSSMD0 = 1;                         // disable the flash

   return SPI0DAT;
}