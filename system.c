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

__EEPROM_DATA(0x11, 
        MODBUD_ID, 
        0x80, //EE_BUZZER_LOUD_DURATION
        0x0f, //EE_BUZZER_QUIET_DURATION
        0x80, // EE_BUZZER_INFO_PERIOD
        0xff, // EE_BUZZER_ALARM_PERIOD
        0x01, //EE_BUZZER_ON_OFF_DURATION * 256 ms
        0x04); //EE_BUZZER_ON_OFF_PERIOD * 256 ms

__EEPROM_DATA(0x10, //EE_BUZZER_ESCALADE_TIME
        0x03, // EE_BUZZER_START_DURATION_DIV
        0xff, //EE_EVENT_ACCEPT_TIME
        21, // EE_EVENING_TIME_HOUR
        23, //EE_NIGHT_START_HOUR
        9, // EE_NIGHT_END_HOUR
        11, // EE_MORNING_TIME_HOUR
        0x06); //EE_BLINK_DURATION * 64 ms
__EEPROM_DATA(0x09,//EE_BLINK_PERIOD * 64 ms
        0x00, 
        0x01,// EE_EVENT_COUNT
        0x30,// First event alarm 16 32
        0x20,// min
        0x00,
        0x00,
        0x00);

/* Refer to the device datasheet for information about available
oscillator configurations. */
void ConfigureOscillator(void)
{
    /* TODO Add clock switching code if appropriate.  */

    /* Typical actions in this function are to tweak the oscillator tuning
    register, select new clock sources, and to wait until new clock sources
    are stable before resuming execution of the main project. */
}