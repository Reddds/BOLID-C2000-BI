/******************************************************************************/
/*Files to Include                                                            */
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

#define	TXE_DELAY 	10

#define TIME_NOT_SET 0xFF

static volatile unsigned long millisecondsFromStart = 0;
static volatile uint8_t globalMinutes = 0;
static volatile uint8_t _6sCounter = 0;
static volatile uint8_t globalHours = TIME_NOT_SET;
//static volatile time_t currentTime = 0;


// UART Buffer

#define UART_BUF_LEN 256u
static uint8_t UartBufferHead;
static volatile uint8_t UartBufferTail;
static volatile uint8_t UartBufferLen;
static volatile uint8_t UartRingBuffer[UART_BUF_LEN];

void InitUartBuffer()
{
    UartBufferHead = 0;
    UartBufferTail = 0;
    UartBufferLen = 0;

}


uint8_t PortAvailable()
{
    return UartBufferLen;
}

uint8_t PortRead()
{
    di();
    if(UartBufferLen > 0)
    {
    
        uint8_t ret = UartRingBuffer[UartBufferTail];
        UartBufferLen--;
        if(UartBufferTail == UART_BUF_LEN - 1)
            UartBufferTail = 0;
        else
            UartBufferTail++;
        ei();
        return ret;
    }
    ei();
    return 0;
}

//void SetRS485TxPin(bool value)
//{
//    
//}

void PortWriteByte(uint8_t b)
{
    LATCbits.LATC5 = 1;
    __delay_us(TXE_DELAY);
    while(!TRMT);
    TXREG = b;
    while(!TRMT);
    LATCbits.LATC5 = 0;
}

void PortWrite(uint8_t *buf, uint8_t buflen)
{
    LATCbits.LATC5 = 1;
    __delay_us(TXE_DELAY);
    for(uint8_t i = 0; i < buflen; i++)
    {
        while(!TRMT);
        TXREG = buf[i];
    }
    while(!TRMT);
    LATCbits.LATC5 = 0;
}

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */

#if defined(__XC) || defined(HI_TECH_C)
void interrupt high_isr(void)
#elif defined (__18CXX)
#pragma code high_isr=0x08
#pragma interrupt high_isr
void high_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif

{

      /* This code stub shows general interrupt handling.  Note that these
      conditional statements are not handled within 3 seperate if blocks.
      Do not use a seperate if block for each interrupt flag to avoid run
      time errors. */

#if 1
    
      /* TODO Add High Priority interrupt routine code here. */

      /* Determine which flag generated the interrupt */
      if(PIR1bits.TMR1IF && PIE1bits.TMR1IE) // milliseconds timer
      {
        PIR1bits.TMR1IF = 0; /* Clear Interrupt Flag 1 */
        millisecondsFromStart++; 
        WRITETIMER1(TIMER_TICKS_IN_1_MS);
        return;
      }
      
      if(INTCONbits.TMR0IF) // Watch timer
      {
          INTCONbits.TMR0IF = 0;
          _6sCounter++;
          if(_6sCounter == 10)
          {
              _6sCounter = 0;
              globalMinutes++;
              if(globalMinutes == 60)
              {
                  globalMinutes = 0;
                  globalHours++;
                  if(globalHours == 24)
                  {
                      globalHours = 0;
                  }
              }
          }
          WRITETIMER0(WATCH_TIMER_TICKS_IN_6_S);
          return;
      }

      if (PIR1bits.RCIF && PIE1bits.RCIE)
      {
//        while(!RCIF);
//        uint8_t c = RCREG;
//        UartBufferLen++;
//        PIR1bits.RCIF = 0;
        if(UartBufferLen == UART_BUF_LEN) // if buffer is full
        {
            while(!RCIF);
            PIR1bits.RCIF = 0;
            return;
        }
        
        while(!RCIF);
        uint8_t c = RCREG;
       
        //uint8_t var_head = UartBufferHead;     // we create temporary variable
        UartRingBuffer[UartBufferHead] = c; // save the data in FIFO head
        UartBufferLen++;// increment counter of waiting bytes
        if(UartBufferHead == UART_BUF_LEN - 1)
            UartBufferHead = 0;
        else
            UartBufferHead++;
        
        // reset interrupt
        PIR1bits.RCIF = 0;
        return;
      }

#endif

}

/* Low-priority interrupt routine */
#if defined(__XC) || defined(HI_TECH_C)
void low_priority interrupt low_isr(void)
#elif defined (__18CXX)
#pragma code low_isr=0x18
#pragma interruptlow low_isr
void low_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif
{

      /* This code stub shows general interrupt handling.  Note that these
      conditional statements are not handled within 3 seperate if blocks.
      Do not use a seperate if block for each interrupt flag to avoid run
      time errors. */

#if 0

      /* TODO Add Low Priority interrupt routine code here. */

      /* Determine which flag generated the interrupt */
      if(<Interrupt Flag 1>)
      {
          <Interrupt Flag 1=0>; /* Clear Interrupt Flag 1 */
      }
      else if (<Interrupt Flag 2>)
      {
          <Interrupt Flag 2=0>; /* Clear Interrupt Flag 2 */
      }
      else
      {
          /* Unhandled interrupts */
      }

#endif

}

unsigned long millis()
{
    // millisecondsFromStart can changed while read;
    di();
    unsigned long ret = millisecondsFromStart;
    ei();
    return ret;
}

bool getHourMin(uint8_t *hour, uint8_t *min)
{
    if(globalHours == TIME_NOT_SET)
        return false;
    di();
    *hour = globalHours;
    *min = globalMinutes;
    ei();
    return true;
}

void SetHourMin(int *newHour, int *newMin)
{
    globalHours = *newHour;
    globalMinutes = *newMin;
}
