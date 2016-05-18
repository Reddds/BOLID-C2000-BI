#ifndef SYSTEM_H
#define	SYSTEM_H
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
#include <xc.h>

/* TODO Define system operating frequency */
#pragma config IDLOC0 = 0 //major
#pragma config IDLOC1 = 1 //minor
#pragma config IDLOC2 = 0 //rev

// HS oscillator

#pragma config  OSC            = HS    

#pragma config  WDT             = OFF
#pragma config  BOR             = ON
#pragma config  BORV            = 42

#pragma config	CP0 			= OFF			//;Code Protect 00800-03FFF
#pragma config	CP1 			= OFF			//;Code Protect 04000-07FFF
#pragma config	CP2             = OFF			//;Code Protect 08000-0BFFF
#pragma config	CP3 			= OFF			//;Code Protect 0C000-0FFFF
#pragma config	CPB 			= OFF			//;Code Protect Boot
#pragma config	CPD 			= OFF			//;Data EE Read Protect
#pragma config	WRT0 			= OFF			//;Table Write Protect 00800-03FFF
#pragma config	WRT1 			= OFF			//;Table Write Protect 04000-07FFF
#pragma config	WRT2 			= OFF			//;Table Write Protect 08000-0BFFF
#pragma config	WRT3 			= OFF			//;Table Write Protect 0C000-0FFFF
#pragma config	WRTC 			= OFF			//;Config. Write Protect
#pragma config	WRTB 			= OFF			//;Table Write Protect Boot
#pragma config	WRTD 			= OFF			//;Data EE Write Protect
#pragma config	EBTR0 			= OFF			//;Table Read Protect 00800-03FFF		
#pragma config	EBTR1 			= OFF			//;Table Read Protect 04000-07FFF
#pragma config	EBTR2 			= OFF			//;Table Read Protect 08000-0BFFF
#pragma config	EBTR3			= OFF			//;Table Read Protect 0C000-0FFFF
#pragma config	EBTRB 			= OFF       	//;Table Read Protect Boot

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        10000000L
#define _XTAL_FREQ      SYS_FREQ
#define FCY             SYS_FREQ/4

#define TIMER_TICKS_IN_1_MS 0x10000 - FCY / 1000

// 1 sec = FCY = 2 500 000 
// 1 min = FCY * 60 = 150000000
// 1/10 min = FCY * 6 = 15 000 000
#define WATCH_TIMER_PRESCALER 256
#define WATCH_TIMER_CORRECTION 0

#define WATCH_TIMER_TICKS_IN_1_SEC (FCY / WATCH_TIMER_PRESCALER + WATCH_TIMER_CORRECTION)
#define WATCH_TIMER_TICKS_TO_END_6S 0x10000 - WATCH_TIMER_TICKS_IN_1_SEC * 6

#define MODBUD_ID       10



#define false 0
#define true 1

#define word(hb, lb) (((uint16_t)hb<<8)|lb)
#define byteFrom2(hb, lb) (((hb << 4) & 0xf0) | (lb & 0x0f))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= ((unsigned short)1 << (bit))) // (1UL << (bit))
#define bitClear(value, bit) ((value) &= ~((unsigned short)1 << (bit))) // (1UL << (bit))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit)) 

typedef char int8_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;

typedef unsigned char boolean;
typedef unsigned char bool;

#define EE_MODBUS_ID 1

/*
define HOLDING_BUZZER_LOUD_QUIET_DURATION 1 // Day|Evening buzzer duration
#define HOLDING_BUZZER_INFO_ALARM_PERIOD 2 // HI - Info Period, LO - Alarm period
#define HOLDING_BUZZER_ON_OFF_DURATION_PERIOD 3 // * 256 ms

#define HOLDING_EVENT_ACCEPT_TIME_S 4  // Time to react to the event in sec

#define HOLDING_BUZZER_ESCALADE 5 // HI - Time from start playing to full loud, LO - Start duration divider(Duration >> x)

#define HOLDING_EVENING_TIME_HOUR_MIN 6 // Quiet buzzer
#define HOLDING_NIGHT_START_HOUR_MIN 7 // Mute buzzer
#define HOLDING_NIGHT_END_HOUR_MIN 8 // Quiet buzzer
#define HOLDING_MORNING_TIME_HOUR_MIN 9 // Loud buzzer start

#define HOLDING_BLINK_DURATION_PERIOD 10 // HI - duration, ms << 6, LO - Period, ms << 6

 
 */

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */


void ConfigureOscillator(void); /* Handles clock switching/osc initialization */

#endif

