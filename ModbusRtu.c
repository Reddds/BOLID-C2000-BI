#if defined(__XC)
#include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
#include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
#include <p18cxxx.h>    /* C18 General Include File */
#endif

#include <eeprom_routines.h>

#include "ModbusRtu.h"
#include "user.h"
#include "interrupts.h"

typedef struct
{
    uint8_t u8id; /*!< Slave address between 1 and 247. 0 means broadcast */
    uint8_t u8fct; /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd; /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo; /*!< Number of coils or registers to access */
    uint16_t *au16reg; /*!< Pointer to memory image in master */
}
modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
    ID = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT //!< byte counter = 6
};

enum FILE_MESSAGE
{
    FILE_DATA_LEN = 2, //!< Request data length
    FILE_REF_TYPE, //!< Reference Type
    FILE_NUM_HI, //!< File Number high byte
    FILE_NUM_LO, //!< File Number low byte
    FILE_REC_HI, //!< Record Number high byte
    FILE_REC_LO, //!< Record Number low byte
    FILE_REC_LEN_HI, //!< Record length high byte
    FILE_REC_LEN_LO, //!< Record length low byte
    FILE_FIRST_BYTE
};

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
    MB_FC_WRITE_MULTIPLE_COILS = 15,        /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16,    /*!< FCT=16 -> write multiple registers */
    MB_FC_REPORT_SLAVE_ID = 17,             /*!< FCT=17 -> Report Slave ID */
    MB_FC_WRITE_FILE_RECORD = 21            // (0x15) Write File Record
};

enum COM_STATES
{
    COM_IDLE = 0,
    COM_WAITING = 1

};

enum ERR_LIST
{
    ERR_NOT_MASTER = -1,
    ERR_POLLING = -2,
    ERR_BUFF_OVERFLOW = -3,
    ERR_BAD_CRC = -4,
    ERR_EXCEPTION = -5
};

enum
{
    NO_REPLY = 255,
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

const unsigned char fctsupported[] = {
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS,
    MB_FC_REPORT_SLAVE_ID,
    MB_FC_WRITE_FILE_RECORD
};

#define T35  5
#define MAX_BUFFER  140	//!< maximum size for the communication buffer in bytes


uint8_t _u8id; //!< 0=master, 1..247=slave number
uint8_t _u8serno; //!< serial port: 0-Serial, 1..3-Serial1..Serial3
uint8_t _u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
uint8_t _u8state;
uint8_t _u8lastError;
uint8_t _au8Buffer[MAX_BUFFER];
uint8_t _u8BufferSize;
uint8_t _u8lastRec;
//uint16_t *_holdingRegs;
uint8_t _inputRegsCount, _holdingRegsCount;
uint16_t _u16InCnt, _u16OutCnt, _u16errCnt;
uint16_t _u16timeOut;
uint32_t _u32time, _u32timeOut;

MODBUS_COMMANDS_t _lastCommand = 0;
uint16_t _lastAddress = 0;
uint16_t _lastCount = 0; // number of coils or registers or file length in bytes in last command 

void ModbusInit(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);
void ModbusSendTxBuffer();
int8_t ModbusGetRxBuffer();
uint16_t ModbusCalcCRC(uint8_t u8length);
uint8_t ModbusValidateAnswer();
uint8_t ModbusValidateRequest();
void ModbusGet_FC1();
void ModbusGet_FC3();
int8_t ModbusProcess_FC1(uint16_t regs); // Read Coils
int8_t ModbusProcess_FC3(uint16_t *regs, uint8_t u8size); // Read Holding Registers
int8_t ModbusProcess_FC5(uint16_t *regs); // Write Single Coil &regs 
int8_t ModbusProcess_FC6(uint16_t *regs, uint8_t u8size); // Write Single Register
int8_t ModbusProcess_FC15(uint16_t *regs); //Write Multiple Coils&regs 
int8_t ModbusProcess_FC16(uint16_t *regs, uint8_t u8size); //Write Multiple registers
int8_t ModbusProcess_FC17(); //Report Slave ID
// Writing to EEPROM
int8_t ModbusProcess_FC21(); //Write File Record
void ModbusBuildException(uint8_t u8exception); // build exception message
/* _____PUBLIC FUNCTIONS_____________________________________________________ */

/**
 * @brief
 * Full constructor for a Master/Slave through USB/RS232C/RS485
 * It needs a pin for flow control only for RS485 mode
 *
 * @param u8id   node address 0=master, 1..247=slave
 * @param u8serno  serial port used 0..3
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 * @overload Modbus::Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
 * @overload Modbus::Modbus()
 */

Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
{
    ModbusInit(u8id, u8serno, u8txenpin);
}


/**
 * @brief
 * Initialize class object.
 *
 * Sets up the serial port using specified baud rate.
 * Call once class has been instantiated, typically within setup().
 *
 * @see http://arduino.cc/en/Serial/Begin#.Uy4CJ6aKlHY
 * @param speed   baud rate, in standard increments (300..115200)
 * @param config  data frame settings (data length, parity and stop bits)
 * @ingroup setup
 */
#ifdef ARDUINO

void Modbus::begin(long u32speed)
{

    switch (u8serno)
    {
#if defined(UBRR1H)
        case 1:
            port = &Serial1;
            break;
#endif

#if defined(UBRR2H)
        case 2:
            port = &Serial2;
            break;
#endif

#if defined(UBRR3H)
        case 3:
            port = &Serial3;
            break;
#endif
        case 0:
        default:
            port = &Serial;
            break;
    }

    // port->begin(u32speed, u8config);
    port->begin(u32speed);
    if (u8txenpin > 1)
    { // pin 0 & pin 1 are reserved for RX/TX
        // return RS485 transceiver to transmit mode
        pinMode(u8txenpin, OUTPUT);
        digitalWrite(u8txenpin, LOW);
    }

    port->flush();
    u8lastRec = u8BufferSize = 0;
    u16InCnt = u16OutCnt = u16errCnt = 0;


}
#elif __XCf 

void ModbusBegin()
{
    //PortBegin();
}
#endif
/**
 * @brief
 * Initialize default class object.
 *
 * Sets up the serial port using 19200 baud.
 * Call once class has been instantiated, typically within setup().
 *
 * @overload Modbus::begin(uint16_t u16BaudRate)
 * @ingroup setup
 */
#ifdef ARDUINO

void Modbus::begin()
{
    begin(19200);
}
#endif
/**
 * @brief
 * Method to write a new slave ID address
 *
 * @param 	u8id	new slave address between 1 and 247
 * @ingroup setup
 */
/*
void ModbusSetID( uint8_t u8id_) 
{
  if (( u8id_ != 0) && (u8id_ <= 247)) 
  {
    u8id = u8id_;
  }
}
 */

/**
 * @brief
 * Method to read current slave ID address
 *
 * @return u8id	current slave address between 1 and 247
 * @ingroup setup
 */
/*
uint8_t ModbusGetID()
{
    return _u8id;
}
*/
/**
 * @brief
 * Initialize time-out parameter
 *
 * Call once class has been instantiated, typically within setup().
 * The time-out timer is reset each time that there is a successful communication
 * between Master and Slave. It works for both.
 *
 * @param time-out value (ms)
 * @ingroup setup
 */
/*
void ModbusSetTimeOut(uint16_t u16timeOut)
{
    _u16timeOut = u16timeOut;
}
*/
/**
 * @brief
 * Return communication Watchdog state.
 * It could be usefull to reset outputs if the watchdog is fired.
 *
 * @return TRUE if millis() > u32timeOut
 * @ingroup loop
 */
/*
boolean ModbusGetTimeOutState()
{
    return (millis() > _u32timeOut);
}
*/
/**
 * @brief
 * Get input messages counter value
 * This can be useful to diagnose communication
 *
 * @return input messages counter
 * @ingroup buffer
 */
/*
uint16_t ModbusGetInCnt()
{
    return _u16InCnt;
}
*/
/**
 * @brief
 * Get transmitted messages counter value
 * This can be useful to diagnose communication
 *
 * @return transmitted messages counter
 * @ingroup buffer
 */
/*
uint16_t ModbusGetOutCnt()
{
    return _u16OutCnt;
}
*/
/**
 * @brief
 * Get errors counter value
 * This can be useful to diagnose communication
 *
 * @return errors counter
 * @ingroup buffer
 */
/*
uint16_t ModbusGetErrCnt()
{
    return _u16errCnt;
}
*/
/**
 * Get modbus master state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
/*
uint8_t ModbusGetState()
{
    return _u8state;
}
*/
/**
 * Get the last error in the protocol processor
 *
 * @returnreturn   NO_REPLY = 255      Time-out
 * @return   EXC_FUNC_CODE = 1   Function code not available
 * @return   EXC_ADDR_RANGE = 2  Address beyond available space for Modbus registers
 * @return   EXC_REGS_QUANT = 3  Coils or registers number beyond the available space
 * @ingroup buffer
 */
/*
uint8_t ModbusGetLastError()
{
    return _u8lastError;
}
*/
/**
 * @brief
 * *** Only for Modbus Slave ***
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the Master and the Slave, the time-out timer is reset.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return 0 if no query, 1..4 if communication error, >4 if correct query processed
 * @ingroup loop
 */

uint8_t ModbusPoll(uint16_t discreteInputs, uint16_t *coils, uint16_t *inputRegs, const uint8_t inputRegsCount,
        uint16_t *holdingRegs, const uint8_t holdingRegsCount)
{
    _lastCommand = MODBUS_COM_NONE;
    //_inputRegs = inputRegs;
    //_holdingRegs = holdingRegs;
    _inputRegsCount = inputRegsCount;
    _holdingRegsCount = holdingRegsCount;


    // check if there is any incoming frame

    uint8_t u8current = PortAvailable();



    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != _u8lastRec)
    {
        _u8lastRec = u8current;
        _u32time = millis() + T35;
        return 0;
    }
    if (millis() < _u32time)
        return 0;

    _u8lastRec = 0;
    int8_t i8state = ModbusGetRxBuffer();
    _u8lastError = i8state;
    if (i8state < 4) // Minimum request len
        return i8state;

    // check slave id
    if (_au8Buffer[ ID ] != _u8id)
        return 0;

    // validate message: CRC, FCT, address and size
    uint8_t u8exception = ModbusValidateRequest();
    if (u8exception > 0)
    {
        if (u8exception != NO_REPLY)
        {
            ModbusBuildException(u8exception);
            ModbusSendTxBuffer();
        }
        _u8lastError = u8exception;
        return u8exception;
    }

    _u32timeOut = millis() + (long) _u16timeOut;
    _u8lastError = 0;

    // process message
    switch (_au8Buffer[ FUNC ])
    {
        case MB_FC_READ_COILS:
            return ModbusProcess_FC1(*coils);
        case MB_FC_READ_DISCRETE_INPUT:
            return ModbusProcess_FC1(discreteInputs);
        case MB_FC_READ_REGISTERS:
            return ModbusProcess_FC3(holdingRegs, holdingRegsCount);
        case MB_FC_READ_INPUT_REGISTER:
            return ModbusProcess_FC3(inputRegs, inputRegsCount);
        case MB_FC_WRITE_COIL:
            return ModbusProcess_FC5(coils);
        case MB_FC_WRITE_REGISTER:
            return ModbusProcess_FC6(holdingRegs, holdingRegsCount);
        case MB_FC_WRITE_MULTIPLE_COILS:
            return ModbusProcess_FC15(coils);
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return ModbusProcess_FC16(holdingRegs, holdingRegsCount);
        case MB_FC_REPORT_SLAVE_ID:
            return ModbusProcess_FC17();
        case MB_FC_WRITE_FILE_RECORD:
            return ModbusProcess_FC21();
        default:
            break;
    }
    return i8state;
}

/* _____PRIVATE FUNCTIONS_____________________________________________________ */

void ModbusInit(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
{
    _u8id = u8id;
    _u8serno = (u8serno > 3) ? 0 : u8serno;
    _u8txenpin = u8txenpin;
    _u16timeOut = 1000;
}

/**
 * @brief
 * This method moves Serial buffer data to the Modbus au8Buffer.
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
int8_t ModbusGetRxBuffer()
{
    bool bBuffOverflow = false;

    //!!!  if (u8txenpin > 1) 
    //!!!      digitalWrite( u8txenpin, LOW );

    _u8BufferSize = 0;

    while (PortAvailable())
    {
        _au8Buffer[ _u8BufferSize ] = PortRead();

        _u8BufferSize++;

        if (_u8BufferSize >= MAX_BUFFER)
            bBuffOverflow = true;
    }
    _u16InCnt++;

    if (bBuffOverflow)
    {
        _u16errCnt++;
        return ERR_BUFF_OVERFLOW;
    }
    return _u8BufferSize;
}

/**
 * @brief
 * This method transmits au8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param nothing
 * @return nothing
 * @ingroup buffer
 */
void ModbusSendTxBuffer()
{
    //  uint8_t i = 0;

    // append CRC to message
    uint16_t u16crc = ModbusCalcCRC(_u8BufferSize);
    _au8Buffer[ _u8BufferSize ] = u16crc >> 8;
    _u8BufferSize++;
    _au8Buffer[ _u8BufferSize ] = u16crc & 0x00ff;
    _u8BufferSize++;


    // transfer buffer to serial line
    PortWrite(_au8Buffer, _u8BufferSize);

    // keep RS485 transceiver in transmit mode as long as sending


    //!!!  port->flush();
    _u8BufferSize = 0;

    // set time-out for master
    _u32timeOut = millis() + (unsigned long) _u16timeOut;

    // increase message counter
    _u16OutCnt++;
}

// Table CRC -------------------------------------------------------------------
/*
const uint8_t auchCRCHi[256] ={
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

const uint8_t auchCRCLo[256] ={
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};*/
#if 0 

uint16_t ModbusCalcCRC(uint8_t u8length)
{
    uint8_t *p = _au8Buffer;
    uint8_t crc_hi;
    uint8_t crc_lo;
    uint8_t n;

    if (u8length > 256U)
    {
        return (0);
    }

    n = u8length;

    crc_hi = 0xFF; // high byte of CRC initialized
    crc_lo = 0xFF; // low byte of CRC initialized

    do
    {
        uint8_t i = crc_hi ^ *p++; // will index into CRC lookup table
        crc_hi = crc_lo ^ (uint8_t) (&auchCRCHi[i]); // calculate the CRC
        crc_lo = (uint8_t) (&auchCRCLo[i]);
    }    while (--n); // pass through message buffer (max 256 items)

    return ((crc_hi << 8) | crc_lo);
}
#endif
// Table CRC END ---------------------------------------------------------------
#if true

/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
uint16_t ModbusCalcCRC(uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (uint8_t i = 0; i < u8length; i++)
    {
        temp = temp ^ _au8Buffer[i];
        for (uint8_t j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}
#endif

/**
 * @brief
 * This method validates slave incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
uint8_t ModbusValidateRequest()
{
    // check message crc vs calculated crc
    uint16_t u16MsgCRC =
            ((_au8Buffer[_u8BufferSize - 2] << 8)
            | _au8Buffer[_u8BufferSize - 1]); // combine the crc Low & High bytes
    if (ModbusCalcCRC(_u8BufferSize - 2) != u16MsgCRC)
    {
        _u16errCnt++;
        return NO_REPLY;
    }

    // check fct code
    bool isSupported = false;
    for (uint8_t i = 0; i< sizeof ( fctsupported); i++)
    {
        if (fctsupported[i] == _au8Buffer[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        _u16errCnt++;
        return EXC_FUNC_CODE;
    }

    // check start address & nb range
    uint16_t u16regs = 0;
    uint16_t u16count = 0;
    uint8_t u8regs;
    switch (_au8Buffer[ FUNC ])
    {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_WRITE_MULTIPLE_COILS:
            // Всего может быть до 16 адресов
            u16regs = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
            u16count = word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
            if (u16count > 16)
                return EXC_REGS_QUANT;
            if (u16regs > 15 || u16regs + u16count > 16)
                return EXC_ADDR_RANGE;
            break;
        case MB_FC_WRITE_COIL:
            u16regs = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
            u8regs = _au8Buffer[ NB_HI ];
            if (u8regs != 0x00 && u8regs != 0xFF)
                return EXC_REGS_QUANT;
            if (u16regs > 15)
                return EXC_ADDR_RANGE;
            break;
        case MB_FC_WRITE_REGISTER:
            u16regs = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
            u8regs = (uint8_t) u16regs;
            if (u8regs > _holdingRegsCount) return EXC_ADDR_RANGE;
            break;
        case MB_FC_READ_INPUT_REGISTER: // 4
            u16regs = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
            u16regs += word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
            u8regs = (uint8_t) u16regs;
            if (u8regs > _inputRegsCount)
                return EXC_ADDR_RANGE;
            break;
        case MB_FC_READ_REGISTERS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            u16regs = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
            u16regs += word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
            u8regs = (uint8_t) u16regs;
            if (u8regs > _holdingRegsCount)
                return EXC_ADDR_RANGE;
            break;
        case MB_FC_REPORT_SLAVE_ID:
            break;
            // Write to EEPROM
        case MB_FC_WRITE_FILE_RECORD:
            if (_au8Buffer[ FILE_REF_TYPE ] != 6)
                return EXC_ADDR_RANGE;
            // Support only file # 0x0001
            if (_au8Buffer[ FILE_NUM_HI ] != 0x00 || _au8Buffer[ FILE_NUM_LO ] != 0x01)
                return EXC_ADDR_RANGE;
            // Test for EEPROM range
            unsigned long startAddrBytes = ((_au8Buffer[ FILE_REC_HI ] << 8) | _au8Buffer[ FILE_REC_LO ]) << 1;
            unsigned long recLenBytes = ((_au8Buffer[ FILE_REC_LEN_HI ] << 8) | _au8Buffer[ FILE_REC_LEN_LO ]) << 1;

            if (startAddrBytes + recLenBytes >= _EEPROMSIZE)
                return EXC_ADDR_RANGE;
            break;
    }
    return 0; // OK, no exception code thrown
}

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
//uint8_t ModbusValidateAnswer() {
//  // check message crc vs calculated crc
//  uint16_t u16MsgCRC =
//    ((_au8Buffer[_u8BufferSize - 2] << 8)
//    | _au8Buffer[_u8BufferSize - 1]); // combine the crc Low & High bytes
//  if ( ModbusCalcCRC( _u8BufferSize-2 ) != u16MsgCRC ) {
//    _u16errCnt ++;
//    return NO_REPLY;
//  }
//
//  // check exception
//  if ((_au8Buffer[ FUNC ] & 0x80) != 0) {
//    _u16errCnt ++;
//    return ERR_EXCEPTION;
//  }
//
//  // check fct code
//  boolean isSupported = false;
//  for (uint8_t i = 0; i< sizeof( fctsupported ); i++) {
//    if (fctsupported[i] == _au8Buffer[FUNC]) {
//      isSupported = 1;
//      break;
//    }
//  }
//  if (!isSupported) {
//    _u16errCnt ++;
//    return EXC_FUNC_CODE;
//  }
//
//  return 0; // OK, no exception code thrown
//}

/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup buffer
 */
void ModbusBuildException(uint8_t u8exception)
{
    uint8_t u8func = _au8Buffer[ FUNC ]; // get the original FUNC code

    _au8Buffer[ ID ] = _u8id;
    _au8Buffer[ FUNC ] = u8func + 0x80;
    _au8Buffer[ 2 ] = u8exception;
    _u8BufferSize = EXCEPTION_SIZE;
}

// /**
//  * This method processes functions 1 & 2 (for master)
//  * This method puts the slave answer into master data buffer
//  *
//  * @ingroup register
//  * TODO: finish its implementation
//  */
// void Modbus::get_FC1() {
//   uint8_t u8byte, i;
//   u8byte = 0;

//   //  for (i=0; i< au8Buffer[ 2 ] /2; i++) {
//   //    au16regs[ i ] = word(
//   //    au8Buffer[ u8byte ],
//   //    au8Buffer[ u8byte +1 ]);
//   //    u8byte += 2;
//   //  }
// }

// /**
//  * This method processes functions 3 & 4 (for master)
//  * This method puts the slave answer into master data buffer
//  *
//  * @ingroup register
//  */
// void Modbus::get_FC3() {
//   uint8_t u8byte, i;
//   u8byte = 3;

//   for (i=0; i< au8Buffer[ 2 ] /2; i++) {
//     au16regs[ i ] = word(
//     au8Buffer[ u8byte ],
//     au8Buffer[ u8byte +1 ]);
//     u8byte += 2;
//   }
// }

MODBUS_COMMANDS_t *ModbusGetLastCommand(uint16_t *address, uint16_t *count)
{
    if (address != NULL)
        *address = _lastAddress;
    if (count != NULL)
        *count = _lastCount;
    return &_lastCommand;
}

/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t ModbusProcess_FC1(uint16_t regs)
{
    _lastCommand = MODBUS_COM_READ_COILS;

    //uint8_t u8currentRegister;
    uint8_t u8currentBit, u8bytesno, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
    _lastAddress = u16StartCoil;
    uint16_t u16Coilno = word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
    _lastCount = u16Coilno;
    // put the number of bytes in the outcoming message
    u8bytesno = (uint8_t) (u16Coilno / 8);
    if (u16Coilno % 8 != 0)
        u8bytesno++;
    _au8Buffer[ ADD_HI ] = u8bytesno;
    _u8BufferSize = ADD_LO;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {
        u16coil = u16StartCoil + u16currentCoil;
        //u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) u16coil;

        bitWrite(
                _au8Buffer[ _u8BufferSize ],
                u8bitsno,
                bitRead(regs, u8currentBit));
        u8bitsno++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            _u8BufferSize++;
        }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0)
        _u8BufferSize++;
    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t ModbusProcess_FC3(uint16_t *regs, uint8_t u8size)
{
    _lastCommand = MODBUS_COM_READ_HOLDING;

    uint8_t u8StartAdd = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
    _lastAddress = u8StartAdd;
    uint16_t u16regsno = word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
    _lastCount = u16regsno;
    uint8_t u8CopyBufferSize;
    uint8_t i;

    _au8Buffer[ 2 ] = u16regsno * 2;
    _u8BufferSize = 3;

    for (i = u8StartAdd; i < u8StartAdd + u16regsno; i++)
    {
        _au8Buffer[ _u8BufferSize ] = HIGH_BYTE(regs[i]);
        _u8BufferSize++;
        _au8Buffer[ _u8BufferSize ] = LOW_BYTE(regs[i]);
        _u8BufferSize++;
    }
    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t ModbusProcess_FC5(uint16_t *regs)
{
    _lastCommand = MODBUS_COM_WRITE_SINGLE_COIL;

    //uint8_t u8currentRegister,
    uint8_t u8currentBit;
    uint8_t u8CopyBufferSize;
    uint16_t u16coil = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
    _lastAddress = u16coil;
    _lastCount = 1;
    // point to the register and its bit
    //u8currentRegister = (uint8_t) (u16coil / 16);
    u8currentBit = (uint8_t) (u16coil % 16);

    // write to coil
    bitWrite(
            *regs,
            u8currentBit,
            _au8Buffer[ NB_HI ] == 0xff);


    // send answer to master
    _u8BufferSize = 6;
    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t ModbusProcess_FC6(uint16_t *regs, uint8_t u8size)
{
    _lastCommand = MODBUS_COM_WRITE_SINGLE_REGISTER;

    uint16_t u16add = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
    _lastAddress = u16add;
    _lastCount = 1;
    uint8_t u8CopyBufferSize;
    uint16_t u16val = word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);

    regs[ u16add ] = u16val;

    // keep the same header
    _u8BufferSize = RESPONSE_SIZE;

    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 15 Write Multiple Coils
 * This method writes a bit array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t ModbusProcess_FC15(uint16_t *regs)
{
    _lastCommand = MODBUS_COM_WRITE_MULTIPLE_COILS;
    //   uint8_t u8currentRegister,
    uint8_t u8currentBit, u8frameByte, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;
    boolean bTemp;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word(_au8Buffer[ ADD_HI ], _au8Buffer[ ADD_LO ]);
    _lastAddress = u16StartCoil;
    uint16_t u16Coilno = word(_au8Buffer[ NB_HI ], _au8Buffer[ NB_LO ]);
    _lastCount = u16Coilno;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {

        u16coil = u16StartCoil + u16currentCoil;
        //u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) u16coil;

        bTemp = bitRead(
                _au8Buffer[ u8frameByte ],
                u8bitsno);

        bitWrite(
                *regs,
                u8currentBit,
                bTemp);

        u8bitsno++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    _u8BufferSize = 6;
    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t ModbusProcess_FC16(uint16_t *regs, uint8_t u8size)
{
    _lastCommand = MODBUS_COM_WRITE_MULTIPLE_REGISTERS;
    //  uint8_t u8func = au8Buffer[ FUNC ];  // get the original FUNC code
    uint16_t u16StartAdd = _au8Buffer[ ADD_HI ] << 8 | _au8Buffer[ ADD_LO ];
    _lastAddress = u16StartAdd;
    int8_t u8regsno = _au8Buffer[ NB_HI ] << 8 | _au8Buffer[ NB_LO ];
    _lastCount = u8regsno;
    uint8_t u8CopyBufferSize;
    uint8_t i;
    uint16_t temp;

    // build header
    _au8Buffer[ NB_HI ] = 0;
    _au8Buffer[ NB_LO ] = u8regsno;
    _u8BufferSize = RESPONSE_SIZE;

    // write registers
    for (i = 0; i < u8regsno && (u16StartAdd + i) < u8size; i++) // 
    {
        temp = word(
                _au8Buffer[ (BYTE_CNT + 1) + i * 2 ],
                _au8Buffer[ (BYTE_CNT + 2) + i * 2 ]);

        regs[ u16StartAdd + i ] = temp;
    }
    u8CopyBufferSize = _u8BufferSize + 2;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 17
 * Report Slave ID
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t ModbusProcess_FC17()
{
    _au8Buffer[ 2 ] =  sizeof(SLAVE_ID_STRING) + 1 + 1; // 22 Run Indicator Status 1 Byte + Additional Data 1 Byte

    _u8BufferSize = 3;

    for(uint8_t i = 0; i < sizeof(SLAVE_ID_STRING); i++, _u8BufferSize++)
        _au8Buffer[_u8BufferSize] = SLAVE_ID_STRING[i];

//    _au8Buffer[_u8BufferSize++] = 'P';
//    _au8Buffer[_u8BufferSize++] = 'r';
//    _au8Buffer[_u8BufferSize++] = 'o';
//    _au8Buffer[_u8BufferSize++] = 'v';
//    _au8Buffer[_u8BufferSize++] = 'e';
//    _au8Buffer[_u8BufferSize++] = 'r';
//    _au8Buffer[_u8BufferSize++] = 'k';
//    _au8Buffer[_u8BufferSize++] = 'a';
//
//    _au8Buffer[_u8BufferSize++] = ' ';
//    _au8Buffer[_u8BufferSize++] = 'd';
//    _au8Buffer[_u8BufferSize++] = 'e';
//    _au8Buffer[_u8BufferSize++] = 'l';
//    _au8Buffer[_u8BufferSize++] = 'o';
//    _au8Buffer[_u8BufferSize++] = 'm';


    _au8Buffer[_u8BufferSize++] = '!';
    _au8Buffer[_u8BufferSize++] = '>';
    uint8_t u8CopyBufferSize = _u8BufferSize;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 21
 * This method writes a word array to EEPROM assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t ModbusProcess_FC21()
{
    _lastCommand = MODBUS_COM_WRITE_FILE_RECORD;
    //uint8_t u8func = au8Buffer[ FUNC ];  // get the original FUNC code

    int8_t requestDataLen = _au8Buffer[ FILE_DATA_LEN ];

    uint16_t startAddrsBytes = (word(_au8Buffer[ FILE_REC_HI ], _au8Buffer[ FILE_REC_LO ])) << 1;
    _lastAddress = startAddrsBytes;
    uint16_t recLenBytes = (word(_au8Buffer[ FILE_REC_LEN_HI ], _au8Buffer[ FILE_REC_LEN_LO ])) << 1;
    _lastCount = recLenBytes;
    uint8_t u8CopyBufferSize;
    uint8_t i;
    //  uint16_t temp;

    // build header
    //au8Buffer[ NB_HI ]   = 0;
    //au8Buffer[ NB_LO ]   = u8regsno;
    _u8BufferSize = requestDataLen + 1;


    // write EEPROM
    for (i = 0; i < recLenBytes; i++)
    {
        _EEREG_EEPROM_WRITE(startAddrsBytes + i, _au8Buffer[ FILE_FIRST_BYTE + i ]);
    }

    u8CopyBufferSize = _u8BufferSize; // +2;
    ModbusSendTxBuffer();

    return u8CopyBufferSize;
}

