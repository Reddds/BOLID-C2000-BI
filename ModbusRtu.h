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

#define SLAVE_ID_STRING "Tablo s raspisaniem"


#define MODBUS_RESULT_SUCCESS 0x8080 // Result after executing user code

#define INPUT_REG_LAST_COMMAND_STATE 0 // State after execution last command 0x8080 - All Right






typedef enum  
{
    MODBUS_COM_NONE = 0, // No one command is executed
    MODBUS_COM_READ_COILS = 1,
    MODBUS_COM_READ_HOLDING = 3,
    MODBUS_COM_WRITE_SINGLE_COIL = 5,
    MODBUS_COM_WRITE_SINGLE_REGISTER,
    MODBUS_COM_WRITE_MULTIPLE_COILS = 15,
    MODBUS_COM_WRITE_MULTIPLE_REGISTERS = 16,
    MODBUS_COM_WRITE_FILE_RECORD = 21
} MODBUS_COMMANDS_t;

//  Modbus();
//  Modbus(uint8_t u8id, uint8_t u8serno);
  Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);
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
  MODBUS_COMMANDS_t *ModbusGetLastCommand(uint16_t *address, uint16_t *count);


