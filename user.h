#ifndef USER_H
#define	USER_H
#include <time.h>
/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

//#define MINUTES_IN_DAY 1440
//#define MINUTES_NOT_SET 0xFFFF

//#define HOUR_NOT_SET 0xFF


#define modbusInputBufLen 15
#define modbusHoldingBufLen 15 // Led statuses
uint16_t _MODBUSDiscreteInputs = 0;
uint16_t _MODBUSCoils = 0;
uint16_t _MODBUSInputRegs[modbusInputBufLen];
uint16_t _MODBUSHoldingRegs[modbusHoldingBufLen];
/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

#define ValueShift(v, shift) (v <<= shift)
#define NotShift
#define MbRegWithin(rAddr) (rAddr >= lastAddress && rAddr <= lastEndAddress)

#define LoadMbValueAndSaveEeprom(modbusvalue, v, eeAddr, shift)\
            v = modbusvalue; \
            _EEREG_EEPROM_WRITE(eeAddr, v); \
            shift;

#define LoadMbValuesWithSelectableShift(modbusRegister, v1, shift1, v2, shift2, eeAddr1, eeAddr2)     \
        if(MbRegWithin(modbusRegister))\
        {\
            LoadMbValueAndSaveEeprom(HIGH_BYTE(_MODBUSHoldingRegs[modbusRegister]), v1, eeAddr1, shift1)\
            LoadMbValueAndSaveEeprom(LOW_BYTE(_MODBUSHoldingRegs[modbusRegister]), v2, eeAddr2, shift2)\
            _MODBUSInputRegs[INPUT_REG_LAST_COMMAND_STATE] = MODBUS_RESULT_SUCCESS;\
        }

#define LoadMbUint8Values(modbusRegister, v1, v2, eeAddr1, eeAddr2)   \
        LoadMbValuesWithSelectableShift(modbusRegister, v1, NotShift, v2, NotShift, eeAddr1, eeAddr2);
#define LoadMbValuesWithShift(modbusRegister, v1, shift1, v2, shift2, eeAddr1, eeAddr2)   \
        LoadMbValuesWithSelectableShift(modbusRegister, v1, ValueShift(v1, shift1), v2, ValueShift(v2, shift2), eeAddr1, eeAddr2);

#define LoadMbHiValueWithShift(modbusRegister, v, eeAddr, shift)\
        if(MbRegWithin(modbusRegister))\
        {\
            LoadMbValueAndSaveEeprom(HIGH_BYTE(_MODBUSHoldingRegs[modbusRegister]), v, eeAddr, ValueShift(v, shift))\
            _MODBUSInputRegs[INPUT_REG_LAST_COMMAND_STATE] = MODBUS_RESULT_SUCCESS;\
        }
#define LoadMbHiValue(modbusRegister, v, eeAddr)\
        if(MbRegWithin(modbusRegister))\
        {\
            LoadMbValueAndSaveEeprom(HIGH_BYTE(_MODBUSHoldingRegs[modbusRegister]), v, eeAddr, NotShift)\
            _MODBUSInputRegs[INPUT_REG_LAST_COMMAND_STATE] = MODBUS_RESULT_SUCCESS;\
        }

#define LoadMbLoValueWithShift(modbusRegister, v, eeAddr, shift)\
        if(MbRegWithin(modbusRegister))\
        {\
            LoadMbValueAndSaveEeprom(LOW_BYTE(_MODBUSHoldingRegs[modbusRegister]), v, eeAddr, ValueShift(v, shift))\
            _MODBUSInputRegs[INPUT_REG_LAST_COMMAND_STATE] = MODBUS_RESULT_SUCCESS;\
        }
#define LoadMbLoValue(modbusRegister, v, eeAddr)\
        if(MbRegWithin(modbusRegister))\
        {\
            LoadMbValueAndSaveEeprom(LOW_BYTE(_MODBUSHoldingRegs[modbusRegister]), v, eeAddr, NotShift)\
            _MODBUSInputRegs[modbusRegister] = _MODBUSHoldingRegs[modbusRegister]; /* For debug*/\
            _MODBUSInputRegs[INPUT_REG_LAST_COMMAND_STATE] = MODBUS_RESULT_SUCCESS;\
        }

//void SetRS485TxPin(bool value);

void InitApp(void);         /* I/O and Peripheral Initialization */

void SetTime(time_t *newTime);
//void SetHourMin(int *newHour, int *newMin, int *seconds);
void AddSecond();
time_t *GetTime();
//uint8_t *GetHour();
//uint8_t *GetMinute();
//uint16_t *GetMinutesFromMidnight();
//void SetMinutesFromMidnight(uint16_t minutes);
#endif
