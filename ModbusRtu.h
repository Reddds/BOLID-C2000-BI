/*
 * @file 		ModbusRtu.h
 * @version     1.20
 * @date        2014.09.09
 * @author 		Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution
 *
 * @description
 *  Arduino library for communicating with Modbus devices
 *  over RS232/USB/485 via RTU protocol.
 *
 *  Further information:
 *  http://modbus.org/
 *  http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *
 * @license
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; version
 *  2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * @defgroup setup Modbus Object Instantiation/Initialization
 * @defgroup loop Modbus Object Management
 * @defgroup buffer Modbus Buffer Management
 * @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
 * @defgroup register Modbus Function Codes for Holding/Input Registers
 *
 */


#include "system.h"

/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */

/**
 * @class Modbus
 * @brief
 * Arduino class library for communicating with Modbus devices over
 * USB/RS232/485 (via RTU protocol).
 */
#define MODBUS_ON 0xff
#define MODBUS_OFF 0x00

#define DEFAULT_MODBUS_ID 0x7F

#define DEVICE_TYPE_INDICATOR 0x01

#define SLAVE_ID_DEVICE_TYPE DEVICE_TYPE_INDICATOR // Indicator
#define SLAVE_ID_DEVICE_SUB_TYPE 0x01
#define SLAVE_ID_DEVICE_REVISION 0x01
#define SLAVE_ID_DEVICE_NUMBER 0x00

#define DEVICE_NEED_TIME_SET 1

#define VENDOR_NAME "BOLID"
#define PRODUCT_CODE "C2000-BI"
#define MAJOR_MINOR_REVISION "1.01"
#define VENDOR_URL "http://bolid.ru"
#define PRODUCT_NAME "Tablo s raspisaniem"
#define MODEL_NAME "MMM"
#define USER_APPLICATION_NAME "Raspisanie"

//#define MODBUS_RESULT_SUCCESS 0x8080 // Result after executing user code

//#define INPUT_REG_LAST_COMMAND_STATE 0 // State after execution last command 0x8080 - All Right

//#define HOLDING_COMMAND 0 // HI - command Lo - data
//#define HOLDING_COMMAND_ADDITIONAL_DATA 1

#define EE_MODBUS_ID 1

#define MB_COMMAND_RESET 0x7F
#define MB_COMMAND_SET_ADDRESS 0x01
#define MB_COMMAND_SET_TIME 0x10

#define MB_EXCEPTION_LAST_COMMAND_STATE 0

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE = 0,                         /*!< null operator */
    MB_FC_READ_COILS = 1,                   /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT = 2,          /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS = 3,               /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER = 4,          /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL = 5,                   /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER = 6,               /*!< FCT=6 -> write single register */
    MB_FC_READ_EXCEPTION_STATUS = 7,
    MB_FC_WRITE_MULTIPLE_COILS = 15,        /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16,    /*!< FCT=16 -> write multiple registers */
    MB_FC_REPORT_SLAVE_ID = 17,             /*!< FCT=17 -> Report Slave ID */
    MB_FC_WRITE_FILE_RECORD = 21,           // (0x15) Write File Record
    MB_FC_READ_DEVICE_ID = 43,               //43 / 14 (0x2B / 0x0E) Read Device Identification
    
    MB_FC_SYSTEM_COMMAND = 100,
    MB_FC_USER_COMMAND = 101,
    MB_FC_READ_DEVICE_STATUS = 102
};


//  Modbus();
//  Modbus(uint8_t u8id, uint8_t u8serno);
  void Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);
  void ModbusBegin(long u32speed);
  //void ModbusBegin();
  void ModbusSetTimeOut( uint16_t u16timeout); //!<write communication watch-dog timer
  uint16_t ModbusGetTimeOut(); //!<get communication watch-dog timer value
  boolean ModbusGetTimeOutState(); //!<get communication watch-dog timer state
//   int8_t query( modbus_t telegram ); //!<only for master
//   int8_t poll(); //!<cyclic poll for master

  int8_t ModbusPoll(uint16_t discreteInputs, uint16_t *coils, uint16_t *inputRegs, const uint8_t inputRegsCount, 
    uint16_t *holdingRegs, const uint8_t holdingRegsCount); //!<cyclic poll for slave

  uint16_t ModbusGetInCnt(); //!<number of incoming messages
  uint16_t ModbusGetOutCnt(); //!<number of outcoming messages
  uint16_t ModbusGetErrCnt(); //!<error counter
  uint8_t ModbusGetID(); //!<get slave ID between 1 and 247
  uint8_t ModbusGetState();
  uint8_t ModbusGetLastError(); //!<get last error message
  void ModbusSetID( uint8_t u8id ); //!<write new ID for the slave
  void ModbusEnd(); //!<finish any communication and release serial communication port
  uint8_t *ModbusGetLastCommand(uint16_t *address, uint16_t *count, uint8_t *command);
  void ModbusSetExceptionStatusBit(uint8_t bitNum, boolean value);
  
  uint8_t *ModbusGetUserCommandId();
  uint8_t *ModbusGetUserCommandData();
  uint16_t ModbusGetUserCommandAdditional1();
  uint8_t *ModbusGetUserCommandAdditional1Hi();
  uint8_t *ModbusGetUserCommandAdditional1Lo();
  uint16_t ModbusGetUserCommandAdditional2();
  uint8_t *ModbusGetUserCommandAdditional2Hi();
  uint8_t *ModbusGetUserCommandAdditional2Lo();
  uint16_t ModbusGetUserCommandAdditional3();
  uint8_t *ModbusGetUserCommandAdditional3Hi();
  uint8_t *ModbusGetUserCommandAdditional3Lo();
