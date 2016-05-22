/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <time.h>
//#include <stdio.h> 
#include <eeprom_routines.h>
#endif

#include <limits.h>

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "ModbusRtu.h"  
#include "interrupts.h"

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/
//convenience macros to convert to and from tm years 
#define  tmYearToCalendar(Y) ((Y) + 1970)  // full four digit year 
#define  CalendarYrToTm(Y)   ((Y) - 1970)
#define  tmYearToY2k(Y)      ((Y) - 30)    // offset is from 2000
#define  y2kYearToTm(Y)      ((Y) + 30)   



#define BUZZER_PIN 2
//Period = 4 * (1/clock speed) * 16 * (scaling value + 1)
#define BEEP_STD_FREQ 150
#define CCP1				TRISCbits.TRISC2
	// set the buzzer state
//#define SetBuzzer(period, dutycyle) PR2 = period; CCPR1L = dutycyle
bool IsBusserOn = false;
#define TurnBuzzer(state)	CCP1CON = state
#define StartBuzzer CCP1 = 0; T2CONbits.TMR2ON = 1; IsBusserOn = true;
#define StopBuzzer T2CONbits.TMR2ON = 0; CCP1 = 1; IsBusserOn = false;
#define BuzzerOff  0x0
#define BuzzerOn   0x0F

#define BUTTON_INTRUSION PORTCbits.RC3
#define BUTTON_RESET PORTCbits.RC1

#define INPUT_TIME_SET 0x00

#define EE_BUZZER_LOUD_DURATION 2
#define EE_BUZZER_QUIET_DURATION 3
#define EE_BUZZER_INFO_PERIOD 4
#define EE_BUZZER_ALARM_PERIOD 5
#define EE_BUZZER_ON_OFF_DURATION 6
#define EE_BUZZER_ON_OFF_PERIOD 7
#define EE_BUZZER_ESCALADE_TIME 8
#define EE_BUZZER_START_DURATION_DIV 9
#define EE_EVENT_ACCEPT_TIME 10
#define EE_EVENING_TIME_HOUR 11
#define EE_NIGHT_START_HOUR 12
#define EE_NIGHT_END_HOUR 13
#define EE_MORNING_TIME_HOUR 14
#define EE_BLINK_DURATION 15
#define EE_BLINK_PERIOD 16

#define MAX_EVENTS 60
#define EE_EVENT_COUNT 18
#define EE_FIRST_EVENT EE_EVENT_COUNT + 1 // 60 events * 2 bytes = 120 bytes

// Sounds
#define EE_SOUNDS_COUNT 140

#define PLAY_INFINITE USHRT_MAX

#define DISCRETE_REG_TIME_SET 0
#define DISCRETE_REG_NEED_TIME_SYNC 1


#define COIL_FIRE 0x00 // FIre alarm
#define COIL_WARNING 0x01 // Warning alarm
#define COIL_ALARM 0x02 // Alarm!
#define COIL_ASSAULT 0x03 // Assault
#define COIL_NOT_RESPONSE 0x04 // NotResponse
#define COIL_BLOCKING 0x05 // Blicking
#define COIL_FAULT 0x06 // Fault
#define COIL_WORKING 0x07 // Working

//  #define COIL_BUTTON_PRESSED 0x08 // When Reset Button Pressed
#define COIL_CLEAR_ALL_EVENTS 0x09 // Clear diary


//#define RESET_COIL 0x0f // When set< reset controller

//#define INPUT_REG_LAST_COMMAND_STATE 0 // State after execution last command 0x8080 - All Right
#define INPUT_REG_CURRENT_HOUR_MIN 1
#define INPUT_REG_EVENT_OLD_CUR_NUM 2
#define INPUT_REG_EVENT_HOUR_MIN 3
#define INPUT_REG_CURRENT_HOUR_MIN_2 4
#define INPUT_REG_SECONDS 5
#define INPUT_REG_SOUND_LEN_IS_PLAYING 6
#define INPUT_REG_PL_LEN_POS_IN_EE 7


//#define HOLDING_REG_SETLED 0 // Set led stste HI - Led number [1..60] Lo - state 
//#define HOLDING_BUZZER_LOUD_QUIET_DURATION 1 // Day|Evening buzzer duration
//#define HOLDING_BUZZER_INFO_ALARM_PERIOD 2 // HI - Info Period, LO - Alarm period
//#define HOLDING_BUZZER_ON_OFF_DURATION_PERIOD 3 // * 256 ms

#define HOLDING_EVENT_ACCEPT_TIME_S 4  // Time to react to the event in sec

#define HOLDING_BUZZER_ESCALADE 5 // HI - Time from start playing to full loud, LO - Start duration divider(Duration >> x)

#define HOLDING_EVENING_MORNING_TIME_HOUR 6 // Quiet buzzer|Loud buzzer start
#define HOLDING_NIGHT_START_END_HOUR 7 // Mute buzzer|Quiet buzzer

#define HOLDING_BLINK_DURATION_PERIOD 8 // HI - duration, ms << 6, LO - Period, ms << 6

//#define HOUR_MIN_HOLDING_REG 9
//#define DAY_SEC_HOLDING_REG 10
//#define YEAR_MONTH_HOLDING_REG 11

//#define HOLDING_ADD_EVENT 13 // HI: 5 bit - alarm(1) | info(0),  0-4 bits - hour | LO: minute


// Custom Commands
#define MB_COMMAND_CLEAR_ALL_EVENTS 0x80
#define MB_COMMAND_ADD_EVENT 0x81
#define MB_COMMAND_SET_LED 0x82
#define MB_COMMAND_SET_STATUS_LED 0x83 // Data - HighBit - On/Off, low 3 bits: FIRE, WARNING, Alarm, Napadeniye, NOT_RESPONSE
// Additional: HI - sound Id, LO playDuration * 256msec 0 - once

#define MB_COMMAND_TEST_SOUND 0x90 // Play sound 2 seconds LO: Period, Additional: duration (10 bit))
#define MB_COMMAND_PLAY_SOUND_NUM 0x91 // Data - sound id, additional data - s

#define D7CLC_PIN RC1

/* i.e. uint8_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/


/*###################################################
 * ?????????? ?????????? ? ?????? ???????????
 */
//#pragma code low_vector=0x18 
//void interrupt_at_low_vector(void) 
//{ 
//_asm GOTO low_isr _endasm 
//} 
//#pragma code /* return to the default code section */ 
//#pragma interruptlow low_isr 
//void low_isr (void)
//{
//  // ??? ????? ??????????? ??????????
//}


/*typedef enum
{
    CUR_EVENT_NOT_PROCESSED, // Current event not processed
    CUR_EVENT_ALARMED, // Event blinking and sounded        
         
}EVENT_PROCESS_STATES_t;
*/
uint16_t modbusState;


//uint16_t buzzerOnOffDuration = 0x100; // 256 ms
//uint16_t buzzerOnOffPeriod = 0x400; // ~1 sec
//uint8_t buzzeLoudDuration;
//uint8_t buzzeQuietDuration;
//uint8_t buzzerInfoPeriod;
//uint8_t buzzerAlarmPeriod;
//uint8_t buzzeEscaladeTime;
//uint8_t buzzerStartDurationDiv;
uint8_t eventAcceptTime;
//uint8_t eveningTimeHour;
//uint8_t nightStartHour;
//uint8_t nightEndHour;
//uint8_t morningTimeHour;

uint8_t eventCount;
uint8_t currentEvent = 0;
uint16_t currentEventMinetesFromMidnight = 0; // Minutes when event signal
uint8_t currentEventType = 0; // Alarm(1) Info(0)
        

//uint8_t curEventHour;
//uint8_t curEventMinute;
uint16_t curEventTotalMinutes;
//uint8_t curEventType; // 0 - info 1 - alarm
uint8_t currentAlarmedEventNum = 0xff;
uint8_t curEventNum = 0xff;

bool _isSoundPlaying = false;
uint8_t _soundCount = 0;
uint8_t _playingSoundSteps = 0;
uint8_t _playingSoundStartPosInEe = 0;
uint8_t _playingSoundCurPos = 0;

uint8_t _nextEventSoundId = 0;
uint16_t _nextEventPlayDuration = 0;
//EVENT_PROCESS_STATES_t curEventProcessState = CUR_EVENT_NOT_PROCESSED;
//uint8_t oldEventEndAlarmHour = HOUR_NOT_SET;
//uint8_t oldEventEndAlarmMinute = 0;
time_t eventResetSecond = 0; // Absolute second? before? to reset event

// diagnose and test
time_t soundTestEnd = 0; // Absolute second? before? to reset event

// Minutes from midnight
//uint16_t *minutes;// = MINUTES_NOT_SET;

uint8_t statusLedState = 0x00;

#define LED_STATUSES_LEN 16 // 15 - status leds

/*
 #define COIL_FIRE 0x00 // FIre alarm
#define COIL_WARNING 0x01 // Warning alarm
#define COIL_ALARM 0x02 // Alarm!
#define COIL_ASSAULT 0x03 // Assault
#define COIL_NOT_RESPONSE 0x04 // NotResponse
#define COIL_BLOCKING 0x05 // Blicking
#define COIL_FAULT 0x06 // Fault
#define COIL_WORKING 0x07 // Working
 */

#define LED_STATUS_FIRE COIL_FIRE
#define LED_STATUS_WARNING COIL_WARNING
#define LED_STATUS_ALARM COIL_ALARM
#define LED_STATUS_ASSAULT COIL_ASSAULT
#define LED_STATUS_NOT_RESPONSE COIL_NOT_RESPONSE
#define LED_STATUS_BLOCKING COIL_BLOCKING
#define LED_STATUS_FAULT COIL_FAULT
#define LED_STATUS_WORK COIL_WORKING

uint8_t ledStatuses[LED_STATUSES_LEN];

uint16_t blinkDuration;
uint16_t blinkPeriod;
// 1 - blink
uint8_t ledBlink[LED_STATUSES_LEN];

void io_poll();
void SetTimeFromRegs(uint16_t *hourMin, uint16_t *daySec, uint16_t *yearMonth);
void LoadNextEvent();
typedef enum  {LED_OFF, LED_GREEN, LED_RED, LED_ORANGE} LED_STATES;


void UpdateLedRegister(uint8_t regIndex)
{
    switch(regIndex)
    {
        case 0: // D3
            LATAbits.LATA4 = 0;
            TRISAbits.RA4 = 0; 
            LATAbits.LATA4 = 1;
            TRISAbits.RA4 = 1; 
            break;
        case 1: // D4
            LATCbits.LATC3 = 0;
            TRISCbits.RC3 = 0; 
            LATCbits.LATC3 = 1;
            TRISCbits.RC3 = 1; 
            break;
        case 2: // D6
            LATCbits.LATC0 = 0;
            TRISCbits.RC0 = 0; 
            LATCbits.LATC0 = 1;
            TRISCbits.RC0 = 1; 
            break;
        case 3: // D7
            LATCbits.LATC1 = 0;
            TRISCbits.RC1 = 0; 
            LATCbits.LATC1 = 1;
            TRISCbits.RC1 = 1; 
            break;
    }
}

// Excluding statuses
void SwitchOffAllLeds()
{
    for(uint8_t i = 0; i < LED_STATUSES_LEN - 1; i++)
    {
        ledStatuses[i] = 0;
        ledBlink[i] = 0;
    }
}
// 5 columns 0-5
// 12 rows 0 - 11
// ledNum [1..60]
void LightLed(uint8_t ledNum, LED_STATES ledState, bool blink)
{
    if(ledNum < 1 || ledNum > 60)
        return;
    ledNum--; // zerobase
    
    uint8_t ststusIndex = ledNum >> 2;
    uint8_t statusShift = (ledNum & 0x03) << 1;
    
    switch(ledState)
    {
        case LED_OFF:
            bitClear(ledStatuses[ststusIndex], statusShift + 1);
            bitClear(ledStatuses[ststusIndex], statusShift);
            bitClear(ledBlink[ststusIndex], statusShift + 1);
            bitClear(ledBlink[ststusIndex], statusShift);
            break;
        case LED_GREEN:
            bitSet(ledStatuses[ststusIndex], statusShift + 1);
            bitClear(ledStatuses[ststusIndex], statusShift);
            bitWrite(ledBlink[ststusIndex], statusShift + 1, blink);
            bitClear(ledBlink[ststusIndex], statusShift);
            break;
        case LED_RED:
            bitClear(ledStatuses[ststusIndex], statusShift + 1);
            bitSet(ledStatuses[ststusIndex], statusShift);
            bitClear(ledBlink[ststusIndex], statusShift + 1);
            bitWrite(ledBlink[ststusIndex], statusShift, blink);
            break;    
        case LED_ORANGE:
            bitSet(ledStatuses[ststusIndex], statusShift + 1);
            bitSet(ledStatuses[ststusIndex], statusShift);
            bitWrite(ledBlink[ststusIndex], statusShift + 1, blink);
            bitWrite(ledBlink[ststusIndex], statusShift, blink);
            break;    
    }
    
}


// light on|off status diodes
void LightStatusLed(uint8_t row, bool on, bool blink)
{
    if(row > 7)
        return;
    bitWrite(ledStatuses[LED_STATUSES_LEN-1], row, on);
    bitWrite(ledBlink[LED_STATUSES_LEN-1], row, blink);
    bitWrite(_MODBUSCoils, row, on);
//    UpdateStatusLeds();
}


void pwm_init(void) 
{
    // See the Microchip datasheet for information on how to calculate
    // appropriate values for PR2 and the prescaler, and how many bits
    // are available in your duty cycle

    // This sample uses a PR2 value of 255 and a prescaler of 16 to
    // give a PWM frequency of 2441.41Hz, and a maximum resolution of
    // 14 bits. The PWM module only has 10 bits available as a maximum
    // in the hardware, so we will use all 10 bits.

    // Set CCP1 to PWM mode
    CCP1CONbits.CCP1M = 0x0f;

    // Set PR2
    //Period = 4 * (1/SYS_FREQ) * 16 * (scaling value + 1)
    //PWM period=[(PR2)+1]*4*Tosc*(TMR2 preScalevalue)
    // 0x90 3229 ??
    // [144 + 1] * 4 * 1/10 000 000 * 16 = 0,000928 = 1/1077  1/3229
    PR2 = 255;

    // Set the prescaler to 16
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.T2CKPS0 = 0;

    // Set the PWM pin to be an output
    TRISCbits.RC2 = 0;

    // Turn on Timer 2
    //T2CONbits.TMR2ON = 1;
}
/*
void SetBuzzerFreq(uint16_t freq)
{
    
}*/
// 0 .. 3FF
void SetBuzzerDuty(uint16_t dc) 
{
    // PWM duty cycle = (CCPR1L:CCP1CON<5:4>) *
    // TOSC * (TMR2 prescale value)
    //u16 tempValue = 0;
    CCP1CONbits.DC1B = dc & 0x03;
//    CCP1CONbits.DC1B0 = (dc & 0x01) != 0 ? 1 : 0;
//    CCP1CONbits.DC1B1 = (dc & 0x02) != 0 ? 1 : 0;
    //tempValue = dc >> 2;
    CCPR1L = (uint8_t)(dc >> 2);
}



void InitFromEeprom()
{
    eventAcceptTime         = _EEREG_EEPROM_READ(EE_EVENT_ACCEPT_TIME);
    blinkDuration           = ((uint16_t)_EEREG_EEPROM_READ(EE_BLINK_DURATION)) << 6;
    blinkPeriod             = ((uint16_t)_EEREG_EEPROM_READ(EE_BLINK_PERIOD)) << 6;
    
    eventCount              = _EEREG_EEPROM_READ(EE_EVENT_COUNT);
    
//    SetBuzzerDuty(buzzeLoudDuration); //!!!!!
//    PR2 = buzzerAlarmPeriod;
    
    // First 3 sounds - are for diary
    _soundCount = _EEREG_EEPROM_READ(EE_SOUNDS_COUNT);
    if(_soundCount == 0xFF)
        _soundCount = 0;
    
    Modbus(_EEREG_EEPROM_READ(EE_MODBUS_ID), 0, 0);
    SwitchOffAllLeds();

    
    curEventNum = 0xff;
    curEventTotalMinutes = 0;
    currentAlarmedEventNum = 0xff;
    LoadNextEvent();
    
    _MODBUSInputRegs[INPUT_REG_SOUND_LEN_IS_PLAYING] = word(_soundCount, _isSoundPlaying);
}

#define LightBlock(stat, reg)   \
                if(blinkOn)\
                    LATB = ledStatuses[stat];\
                else\
                    LATB = ledStatuses[stat] ^ ledBlink[stat];\
                UpdateLedRegister(reg);


uint8_t currentLedBlock = 0;
unsigned long diffTime;
unsigned long oldBlinkOnTime = 0;
bool blinkOn = false;

void ProcessLightBlock(unsigned long *curMs)
{
    diffTime = *curMs - oldBlinkOnTime;
    if(diffTime > blinkPeriod)
    {
        blinkOn = true;
        oldBlinkOnTime = *curMs;
    }
    else if(blinkOn && diffTime > blinkDuration)
    {
        blinkOn = false;
    }

    // Off all LED common wires
    // new algo
    switch(currentLedBlock)
    {
        case 0:
            LATA &= 0xF0;
            LightBlock(0, 0)
            LightBlock(1, 1)    
            LightBlock(2, 2) 
            LightBlock(12, 3)    
            LATAbits.LATA0 = 1; // Light 0 led group       
            break;
        case 1:
            LATA &= 0xF0;
            LightBlock(3, 0)
            LightBlock(4, 1)    
            LightBlock(5, 2) 
            LightBlock(13, 3)    
            LATAbits.LATA1 = 1;  // Light 1 led group          
            break;
        case 2:
            LATA &= 0xF0;
            LightBlock(6, 0)
            LightBlock(7, 1)    
            LightBlock(8, 2) 
            LightBlock(14, 3)    
            LATAbits.LATA2 = 1; // Light 2 led group        
            break;
        case 3:
            LATA &= 0xF0;
            LightBlock(9, 0)
            LightBlock(10, 1)    
            LightBlock(11, 2) 
            LightBlock(15, 3)    
            LATAbits.LATA3 = 1; // Light 4 led group        
            break;
    }

    currentLedBlock++;
//    if(currentLedBlock == 4)
//        currentLedBlock = 0;
    currentLedBlock &= 0xFB; // Reset therd bit

}


#define IsNowDayTime(hour) (hour >= morningTimeHour && hour < eveningTimeHour)
//#define IsNowEveningMorning(hour) (hour >= eveningTimeHour && hour < morningTimeHour)
#define IsNowNightTime(hour) (hour >= nightStartHour && hour < nightEndHour)


unsigned long _playingEndMs = 0;


void StopPlaying()
{
    _isSoundPlaying = false;
    StopBuzzer;
    
    _MODBUSInputRegs[INPUT_REG_SOUND_LEN_IS_PLAYING] = word(_soundCount, _isSoundPlaying);
}

void SoundPlayNextStep()
{
    if(_playingSoundCurPos >= _playingSoundSteps)
    {
        _playingSoundCurPos = 0;
        if(*GetTime() >= soundTestEnd)
        {
            StopPlaying();
            return;
        }
    }
    _playingEndMs = millis() + word(_EEREG_EEPROM_READ(_playingSoundStartPosInEe + _playingSoundCurPos * 3), 0);
    PR2 = _EEREG_EEPROM_READ(_playingSoundStartPosInEe + _playingSoundCurPos * 3 + 1);
    uint8_t duration = _EEREG_EEPROM_READ(_playingSoundStartPosInEe + _playingSoundCurPos * 3 + 2);
    _playingSoundCurPos++;        
    if(duration == 0 || PR2 == 0)
    {
        StopBuzzer;
        return;
    }
    SetBuzzerDuty(duration);
    StartBuzzer;
}

/*
 * 0 - sound count
 * 1..N - sound addresses
 * N+1..M - sounds data
 * 
 *  Sound data:
 *  0 - sound len
 *  1..K - sound aequense
 *      0 - play time (ms << 8)
 *      1 - period
 *      2 - duration
 *      
 *      period or duration == 0 : silense
 * 
 */

// playDuration : 0 - once
// ff - infinite
// sec
bool PlaySound(uint8_t soundId, uint16_t playDuration)
{
    if(soundId >= _soundCount)
        return false;
    
    if(playDuration == 0)
        soundTestEnd = 0;
    else if(playDuration == PLAY_INFINITE)
        soundTestEnd = ULONG_MAX;
    else 
        soundTestEnd = *GetTime() + playDuration;
    
    uint8_t soundAddr = _EEREG_EEPROM_READ(EE_SOUNDS_COUNT + 1 + soundId);
    if(EE_SOUNDS_COUNT + _soundCount + soundAddr >= _EEPROMSIZE)
        return false;
    
    _playingSoundSteps = _EEREG_EEPROM_READ(EE_SOUNDS_COUNT + 1 + _soundCount + soundAddr); // * 3 bytes
    
    _playingSoundStartPosInEe = EE_SOUNDS_COUNT + 1 + _soundCount + soundAddr + 1;
    _MODBUSInputRegs[INPUT_REG_PL_LEN_POS_IN_EE] = word(_playingSoundSteps, _playingSoundStartPosInEe);
    if(_playingSoundStartPosInEe + _playingSoundSteps * 3 >= _EEPROMSIZE)
        return false;
    
    _playingSoundCurPos = 0;
    _isSoundPlaying = true;
    SoundPlayNextStep();
    
    _MODBUSInputRegs[INPUT_REG_SOUND_LEN_IS_PLAYING] = word(_soundCount, _isSoundPlaying);
    
    return true;
}



// state: true - user pressed reset button
void ResetEvent(bool state)
{
    LightLed(currentAlarmedEventNum + 1, state ? LED_GREEN : LED_RED, false);  
    currentAlarmedEventNum = 0xff;
    eventResetSecond = 0;
    StopPlaying();
    _MODBUSInputRegs[INPUT_REG_EVENT_OLD_CUR_NUM] = word(currentAlarmedEventNum, curEventNum);
    
    //curEventProcessState = CUR_EVENT_NOT_PROCESSED;
}

void LoadNextEvent()
{
    if(eventCount == 0)
        return;
//    uint8_t hour, minute;
    uint16_t totalMinutes;
//    if(!getHourMin(&hour, &minute))
//        return;
    if(!getTotalMinutes(&totalMinutes))
        return;
    
//    uint8_t *hour = GetHour();
//    if(*hour == HOUR_NOT_SET) // time not set 
//    {
//        return;
//    }
//
//    uint8_t *minute = GetMinute();
    do
    {
        if(curEventNum == 0xff)
            curEventNum = 0;
        else 
            curEventNum++;    
        _MODBUSInputRegs[INPUT_REG_EVENT_OLD_CUR_NUM] = word(currentAlarmedEventNum, curEventNum);
        if(curEventNum >= eventCount)
        {
            curEventNum = 0xff;
            curEventTotalMinutes = 0;
            _MODBUSInputRegs[INPUT_REG_EVENT_HOUR_MIN] = word(0, 0);            
            return;
        }
        // HI: 5-7 - alarmDuration, 0-4 bits - hour | LO: 5-7 soundId 0-5 minute
        // alarmDuration:
        // 0 - once
        // 1 - 10 sec
        // 2 - 30 sec
        // 3 - 1 min
        // 4 - 5 min
        // 5 - 12 min
        // 6 - 30 min
        // 7 - infinite
        uint8_t v1 = _EEREG_EEPROM_READ(EE_FIRST_EVENT + curEventNum * 2);
        curEventTotalMinutes = (v1 & 0x1F) * 60;
        //curEventType = bitRead(v1, 5);
        _nextEventPlayDuration = (v1 >> 5);
        switch(_nextEventPlayDuration)
        {
            case 1:
                _nextEventPlayDuration = 10;
                break;
            case 2:
                _nextEventPlayDuration = 30;
                break;
            case 3:
                _nextEventPlayDuration = 60;
                break;
            case 4:
                _nextEventPlayDuration = 60*5;
                break;
            case 5:
                _nextEventPlayDuration = 60*12;
                break;
            case 6:
                _nextEventPlayDuration = 60*30;
                break;
            case 7:
                _nextEventPlayDuration = PLAY_INFINITE;
                break;
        }
        uint8_t v1 = _EEREG_EEPROM_READ(EE_FIRST_EVENT + curEventNum * 2 + 1);        
        curEventTotalMinutes += v1 & 0x3F;
        _nextEventSoundId = v1 >> 6;
        
    }while(curEventTotalMinutes <= totalMinutes);
    _MODBUSInputRegs[INPUT_REG_EVENT_HOUR_MIN] = curEventTotalMinutes;
    
}
// Fires every minute
void ProcessDiary()
{
    // If no event loaded
    if(curEventNum == 0xff)
        return;
    uint16_t totalMinutes;
//    uint8_t hour, minute;
//    if(!getHourMin(&hour, &minute))
//        return;
    if(!getTotalMinutes(&totalMinutes))
        return;
    // If midnight reset all diodes
    if(totalMinutes == 0)
    {
        SwitchOffAllLeds(); //TODO process if event alarmed yet
        curEventNum = 0xff;
        LoadNextEvent();
        _MODBUSInputRegs[INPUT_REG_EVENT_OLD_CUR_NUM] = word(currentAlarmedEventNum, curEventNum);
    }
    
    
    if(curEventTotalMinutes == totalMinutes) // Event is equal
    {
        // old Event Is Alarmd yet
        if(currentAlarmedEventNum != 0xff)
        {
            LightLed(currentAlarmedEventNum + 1, LED_RED, false);  
        }
        
        {
            currentAlarmedEventNum = curEventNum;
            //curEventProcessState == CUR_EVENT_ALARMED;
            LightLed(currentAlarmedEventNum + 1, LED_ORANGE, true);  
            if(_nextEventSoundId != 0)
            {
                PlaySound(_nextEventSoundId - 1, _nextEventPlayDuration);
            }
//            if(curEventType == 0)
//            {
//                LightLed(currentAlarmedEventNum + 1, LED_GREEN, true);  
//            }
//            else
//            {
//                LightLed(currentAlarmedEventNum + 1, LED_RED, true);  
//            }
            eventResetSecond = *GetTime() + eventAcceptTime;
            
            LoadNextEvent();
        }
        _MODBUSInputRegs[INPUT_REG_EVENT_OLD_CUR_NUM] = word(currentAlarmedEventNum, curEventNum);

    }
//        
//        
//        LightStatusLed(LED_STATUS_BLOCKING, false, false);
//        //PrintTime();
//        LightLed(37 + *hour >> 1, LED_GREEN, false);    
//        LightLed(49 + *minutes / 5, LED_GREEN, false);  
//    }
//    else
//    {
//        LightStatusLed(LED_STATUS_BLOCKING, true, true);
//        
//        //PortWrite("Time not set!\n", 14);
//    }

}

void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();

    

    InitFromEeprom();
    

    
    
    

    /* TODO <INSERT USER APPLICATION CODE HERE> */

    unsigned long lastMs = millis();
    //uint8_t counter = 0;
    
    pwm_init();
    
    
    //ledStatuses[0] = 0x15;
    //ledStatuses[7] = 0x29;
    //bool rg = false;
    unsigned long oldBuzzerOnTime = 0;
    uint8_t oldMinute = 0xff;
    //uint16_t lastMinSec = 0; // Secund counter value
    LightStatusLed(LED_STATUS_WORK, true, false);
    LightStatusLed(LED_STATUS_BLOCKING, true, true); // Time not set yet
    while(1)
    {
        unsigned long curMs = millis();
        ProcessLightBlock(&curMs);

        if(_isSoundPlaying && curMs >= _playingEndMs)
        {
            SoundPlayNextStep();
        }
        
        
        if(BUTTON_RESET == 0)
        {
            // reset alarmed event
            if(currentAlarmedEventNum != 0xff)
            {
                ResetEvent(true);
            }
            else
            {
                StopPlaying();
            }
   
        }

        
        if(curMs - lastMs >= 1000)
        {            
            AddSecond();
            
            if(currentAlarmedEventNum != 0xff && *GetTime() >= eventResetSecond)
            {
                ResetEvent(false);
            }
            
            _MODBUSInputRegs[INPUT_REG_SECONDS] = *GetTime();
            uint8_t hour = 0, minute = 0;
            
            if(getHourMin(&hour, &minute) && oldMinute != minute)
            {
                _MODBUSInputRegs[INPUT_REG_CURRENT_HOUR_MIN] = word(hour, minute);
                
                oldMinute = minute;
                ProcessDiary();
            }
            
            _MODBUSInputRegs[INPUT_REG_CURRENT_HOUR_MIN_2] = word(hour, minute);
            
            lastMs = curMs;

        }
        modbusState = ModbusPoll(_MODBUSDiscreteInputs, &_MODBUSCoils, _MODBUSInputRegs, modbusInputBufLen, _MODBUSHoldingRegs, modbusHoldingBufLen);
        io_poll();
    }


}


void SetTimeFromRegs(uint16_t *hourMin, uint16_t *daySec, uint16_t *yearMonth)
{
    struct tm newTime;
    newTime.tm_year = (*yearMonth >> 8) + 100; // since 1900
    newTime.tm_mon = *yearMonth & 0xFF;
    newTime.tm_mday = *daySec >> 8;
    newTime.tm_hour = *hourMin >> 8;
    newTime.tm_min = *hourMin & 0xFF;
    newTime.tm_sec = *daySec & 0xFF;
    SetHourMin(&newTime.tm_hour, &newTime.tm_min, &newTime.tm_sec);
    time_t newRawTime = mktime(&newTime);
    SetTime(&newRawTime);
    LightStatusLed(LED_STATUS_BLOCKING, false, false);
    LoadNextEvent();
}


void SetTimeCommand()
{
    uint16_t hourMin = _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA];
                   
    uint16_t daySec = _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA + 1];
    uint16_t yearMonth = _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA + 2];
    SetTimeFromRegs(&hourMin, &daySec, &yearMonth);
    bitSet(_MODBUSDiscreteInputs, INPUT_TIME_SET);
}

void CommandSetStatusLed()
{
    // Data - 7bit - On/Off, 6bit - blink low 3 bits: FIRE, WARNING, Alarm, Napadeniye, NOT_RESPONSE
    // Additional: HI - sound Id, LO playDuration,sec 0 - once
    uint8_t commandData = LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND]);
    uint8_t led = commandData & 0x07;
    if(led >= LED_STATUS_BLOCKING)
        return;
    if(bitRead(commandData, 7) == 0)
    {
        LightStatusLed(led, false, false);
        StopPlaying();
        return;
    }
    LightStatusLed(led, true, bitRead(commandData, 6));
    PlaySound(HIGH_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]), LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]));
   ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
}

void io_poll() 
{
    uint16_t lastAddress;
    uint16_t lastEndAddress;
    //uint16_t lastCount;
    uint8_t *lastCommand = ModbusGetLastCommand(&lastAddress, &lastEndAddress);
    if(*lastCommand == MB_FC_NONE)
        return;
    
    lastEndAddress += lastAddress - 1;
    
    uint8_t v1;
    if(*lastCommand == MB_FC_WRITE_REGISTER || *lastCommand == MB_FC_WRITE_MULTIPLE_REGISTERS)
    {
        // Command received
        if(lastAddress == HOLDING_COMMAND)
        {
            uint8_t command = HIGH_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND]);
            uint16_t hourMin;
            switch(command)
            {
                case MB_COMMAND_RESET:
                    #asm
                        RESET; 
                    #endasm
                    return;
                case MB_COMMAND_SET_ADDRESS:
                    break;
                case MB_COMMAND_SET_TIME:
                    SetTimeCommand();
                    ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
                    break;
                    
                    
                case MB_COMMAND_CLEAR_ALL_EVENTS:
                    eventCount = 0;
                    _EEREG_EEPROM_WRITE(EE_EVENT_COUNT, 0);
                    ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
                    break;
//                case MB_COMMAND_ADD_EVENT:
//                    // HI: 5 bit - alarm(1) | info(0),  0-4 bits - hour | LO: minute
//                    if(eventCount < MAX_EVENTS)
//                    {
//                        uint8_t eventEeAddr = EE_FIRST_EVENT + (eventCount << 1);
//                        v1 = HIGH_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]);
//                        _EEREG_EEPROM_WRITE(eventEeAddr, v1);
//                        v1 = LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]);
//                        _EEREG_EEPROM_WRITE(eventEeAddr + 1, v1);
//                        // 2 bit - blink status
//
//                        LightLed(eventCount, LED_GREEN, false);
//
//                        eventCount++;
//                        _EEREG_EEPROM_WRITE(EE_EVENT_COUNT, eventCount);
//
//                        ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
//                    }
//                    else
//                        ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, false);
//                    
//                    break;
                case MB_COMMAND_SET_LED:
                    // 0 Holding reg - First byte - Led num [1..60]
                    // Second byte Value 0 - OFF, 1 - GREEN, 2 - RED 
                    v1 = LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]);
                    // 2 bit - blink status
                    LightLed(HIGH_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]), v1 & 0x03, bitRead(v1, 2));
                    _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA] = 0;
                    ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
                    break;   
//                case MB_COMMAND_TEST_SOUND:
//                    SetBuzzerDuty(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]); //!!!!!
//                    PR2 = LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND]);
//                    //soundTestEnd = *GetTime() + 2; // 2 sec
//                    //StartBuzzer;
//                    break;
                case MB_COMMAND_PLAY_SOUND_NUM:                    
                    //soundTestEnd = *GetTime() + _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]; 
                    PlaySound(LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND]), LOW_BYTE(_MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA]));
                    break;       
            
                case MB_COMMAND_SET_STATUS_LED:  
                    CommandSetStatusLed();
                    break; 
            }
            _MODBUSHoldingRegs[HOLDING_COMMAND] = 0;
            _MODBUSHoldingRegs[HOLDING_COMMAND_ADDITIONAL_DATA] = 0;
        }
    }
    

    if(*lastCommand == MB_FC_WRITE_FILE_RECORD)
    {
        InitFromEeprom();
        ModbusSetExceptionStatusBit(MB_EXCEPTION_LAST_COMMAND_STATE, true);
//        for(uint8_t i = 0; i < eventCount && i < MAX_EVENTS; i++)
//            LightLed(i + 1, LED_GREEN, false);
        return;
    }
}


