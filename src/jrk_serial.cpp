
#include "jrk_hardware/jrk_serial.h"

#define WRITE7BITDATA(byte) ((byte)&0x7F)

//#define DEBUG 1

#ifdef DEBUG
#include <cstdio>
#endif

using jrk::Jrk;

Jrk::Jrk(const std::string port/*=""*/, uint32_t baudrate/*=9600*/, uint32_t timeout/*=0*/)
    : _serial(new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout))) {
};

Jrk::Jrk(serial::Serial* serial) : _serial(serial) {

};

Jrk::~Jrk() {};

void Jrk::reset(serial::Serial* serial)
{
    if (_serial.get()->getBaudrate() == 115200) {
        //--- Reset index ---
        printf("-- Reset Index --\n");
        uint8_t write_buffer[4] = { 0, 0, 0, 0 };
        _serial->write(write_buffer, 4);
        _serial->flushOutput();
    }

    _serial.reset(serial);
}

void Jrk::sendBaudRateIndication()
{
    writeByte(baudRateIndication);
}

void Jrk::motorOff()
{
    writeByte(motorOffCommand);
}

void Jrk::setTarget(uint16_t target)
{
    if (_serial.get()->getBaudrate() == 115200) {
#ifdef DEBUG_PWM
        printf(" Set target [%s] %d \n", _serial.get()->getPort().c_str(), target);
        /*

        // Command to launch testing
        uint8_t write_buffer[5];
        write_buffer[0] = 't';
        write_buffer[1] = 't';
        write_buffer[2] = 't';
        write_buffer[3] = 't';
        _serial->write(write_buffer, 4);
        _serial->flushOutput();
        */
        //return;
#endif
    }

    // split target bytes
    uint8_t lowerByte = target & 0x1F;
    uint8_t upperByte = (target >> 5) & 0x7F;

#ifdef DEBUG
    printf(">> setTarget: %04d\n", target);
#endif

    _write_buffer[0] = setTargetCommand + lowerByte;
    _write_buffer[1] = WRITE7BITDATA(upperByte);
    writeBuffer(2);
}

uint16_t Jrk::get(uint8_t command_byte)
{
    uint8_t read_buffer[256];

    if (_serial.get()->getBaudrate() == 115200) {
        size_t bytes = _serial->read(read_buffer, sizeof(read_buffer));
#ifdef DEBUG_PWM
        if (bytes > 0) {
            printf("-- Begin Read -- \n");
            for (size_t t = 0; t < bytes; t++)
                printf("%c", read_buffer[t]);

            printf("-- End Read -- \n");
        }
        else {
            printf("Failed reading\n");
        }
#endif
        return 0;
    }

    writeByte(command_byte);

    return read16BitData();
}

void Jrk::writeByte(uint8_t dataByte)
{
#ifdef DEBUG
    printf(">> writeByte: 0x%02x\n", dataByte);
#endif
    _write_buffer[0] = dataByte;
    writeBuffer(1);
}

void Jrk::write7BitData(uint8_t data)
{
    writeByte(data & 0x7F);
}

void Jrk::writeBuffer(uint8_t size)
{
    _serial->write(_write_buffer, size);
    _serial->flushOutput();
}

uint16_t Jrk::read16BitData()
{
    size_t bytes;
    bytes = _serial->read(_read_buffer, 2);
    if (bytes < 2)
    {
        throw JrkTimeout();
    }

    uint16_t value = (_read_buffer[1] << 8) | (_read_buffer[0] & 0xFF);

#ifdef DEBUG
    printf("<< read16BitData: %04d\n", value);
#endif

    return value;
}
