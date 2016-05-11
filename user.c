/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#endif

#include "system.h"
#include "user.h"
#include "interrupts.h"
#include "ModbusRtu.h"



/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/
//void high_isr(void);
/* <Initialize variables in user.h and insert code for user algorithms.> */


//#define TIMER_TICKS_IN_1_MS_H TIMER_TICKS_IN_1_MS >> 8
//#define TIMER_TICKS_IN_1_MS_L TIMER_TICKS_IN_1_MS & 0xFF



#define BAUDRATE    9600
#define	UBRG	( (((SYS_FREQ / BAUDRATE) / 8) - 1) / 2 )

time_t currentTime = 0;
//time_t nextMinuteSeconds = 60; // When seconds equal this value? adding 1 minute
//uint16_t minutesFromMidnight = MINUTES_NOT_SET;
//uint8_t currentHour = HOUR_NOT_SET;
//uint8_t currentMinute = 0;

void PortBegin()
{
    TXSTA = 0;
    TXSTAbits.TXEN = 1;
    TXSTAbits.BRGH = 1;
    
    SPBRG = UBRG;
    RCSTA = 0;
    RCSTAbits.SPEN = 1;
    RCSTAbits.CREN = 1;
    
    TRISCbits.RC5 = 0;
    LATCbits.LATC5 = 0;
    
    
}


void InitApp(void)
{
    /* TODO Initialize User Ports/Peripherals/Project here */

    /* Setup analog functionality and port direction */

    /* Initialize peripherals */

    /* Configure the IPEN bit (1=on) in RCON to turn on/off int priorities */

    /* ????????? ?????????? */
    INTCONbits.PEIE = 1; // Peripheral Interrupt Enable
    ei(); // Global Interrupt Enable
    RCONbits.IPEN   = 1; // Interrupt Priority Enable
    
    /* Enable interrupts */
    
    // Init Timer 1 ------------------------------------------------------------
    T1CONbits.RD16 = 1;
    T1CONbits.T1SYNC = 0;
    T1CONbits.TMR1CS = 0;
    // FCY = 2 500 000
    T1CONbits.T1CKPS = 0; // Prescaler 1
    
    
    
    PIE1bits.TMR1IE   = 1;  // Enable interrupt by tomer 1 (interrupt) TMR1
    IPR1bits.TMR1IP   = 1;  // Enable interrupt priority TMR1
    WRITETIMER1(TIMER_TICKS_IN_1_MS);
    //TMR1L              = TIMER_TICKS_IN_1_MS_L; // ???? ???? ?? ????? ??????????? ? ???? ??????? ?? 0xFFFF
    //TMR1H              = TIMER_TICKS_IN_1_MS_H;
    PIR1bits.TMR1IF   = 0; // ??????? ???? ?????????? (????? ????? ?? ????????? ? ??????????)
    
    T1CONbits.TMR1ON = 1; // Switch on timer
    //--------------------------------------------------------------------------
    // Init Timer 0 (1 min) ----------------------------------------------------
    T0CONbits.T08BIT = 0; // 16 bit
    T0CONbits.T0CS = 0; //Internal instruction cycle clock (CLKO)
    // FCY = 2 500 000
    T0CONbits.PSA = 0; //Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output
    /*
    111 = 1:256 prescale value
    110 = 1:128 prescale value
    101 = 1:64 prescale value
    100 = 1:32 prescale value
    011 = 1:16 prescale value
    010 = 1:8 prescale value
    001 = 1:4 prescale value
    000 = 1:2 prescale value
     */
    T0CONbits.T0PS = 7;
    
    
    
    INTCONbits.TMR0IE   = 1;  // Enable interrupt by tomer 1 (interrupt) TMR1
    INTCON2bits.TMR0IP   = 1;  // Enable interrupt priority TMR1
    WRITETIMER0(WATCH_TIMER_TICKS_IN_6_S);
    //TMR1L              = TIMER_TICKS_IN_1_MS_L; // ???? ???? ?? ????? ??????????? ? ???? ??????? ?? 0xFFFF
    //TMR1H              = TIMER_TICKS_IN_1_MS_H;
    INTCONbits.TMR0IF   = 0; // ??????? ???? ?????????? (????? ????? ?? ????????? ? ??????????)
    
    T0CONbits.TMR0ON = 1; // Switch on timer
    //--------------------------------------------------------------------------
    
    
    // Usart
    InitUartBuffer();   
    
    PortBegin();
    PIE1bits.RCIE = 1; // 
    /* Make receive interrupt low priority */
    IPR1bits.RCIP = 0;
    
    
    	// Clearing buffers
	for (unsigned char i = 0; i < modbusInputBufLen; i++)
		_MODBUSInputRegs[i] = 0;
//	for (unsigned char i = 0; i < modbusHoldingBufLen; i++)
//		_MODBUSHoldingRegs[i] = 0;
        //TRISB = 0xC0;
    
    //PORTB = 0x3F;
    TRISB = 0; //  All output
    
    // Common backlight
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    TRISAbits.RA0 = 0; // 0 column
    TRISAbits.RA1 = 0; // 1 column
    TRISAbits.RA2 = 0; // 2 column
    TRISAbits.RA3 = 0; // 6 column
    
    TRISCbits.RC3 = 1; // Input S2
    TRISCbits.RC1 = 1; // ResetButton
    TRISCbits.RC0 = 0; // Output

    LATCbits.LATC2 = 0; // Initial value

}

void SetTime(time_t *newTime)
{
    currentTime = *newTime;
}
//void SetHourMin(int *newHour, int *newMin, int *seconds)
//{
//    currentHour = *newHour;
//    currentMinute = *newMin;
//    nextMinuteSeconds = currentTime + (60 - *seconds);
//}

void AddSecond()
{
    currentTime++;
//    if(currentTime >= nextMinuteSeconds)
//    {
//        nextMinuteSeconds = currentTime + 60;
//        currentMinute++;
//        if(currentMinute >= 60)
//        {
//            currentMinute = 0;
//            currentHour++;
//            if(currentHour >= 24)
//                currentHour = 0;
//        }
//    }
}

//uint8_t *GetHour()
//{
//    return &currentHour;
//}
//
//uint8_t *GetMinute()
//{
//    return &currentMinute;
//}
//
time_t *GetTime()
{
    return &currentTime;
}




