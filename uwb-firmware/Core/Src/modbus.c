#include "modbus.h"

uint8_t GenCRC16(uint8_t* buff, size_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t pos = 0;
    uint8_t i = 0;
    uint8_t lo = 0;
    uint8_t hi = 0;

    for (pos = 0; pos < len; pos++)
    {
        crc ^= buff[pos];

        for (i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    lo = crc & 0xFF;
    hi = (crc >> 8) & 0xFF;

    buff[len++] = lo;
    buff[len++] = hi;
    return len;
}
//------------------------------------------------
//              Проверка CRC16                  //
//------------------------------------------------
uint8_t CheckCRC16(uint8_t* buff, size_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t pos = 0;
    uint8_t i = 0;
    uint8_t lo = 0;
    uint8_t hi = 0;

    for (pos = 0; pos < len - 2; pos++)
    {
        crc ^= buff[pos];

        for (i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    lo = crc & 0xFF;
    hi = (crc >> 8) & 0xFF;
    if ((buff[len - 2] == lo) &&
        (buff[len - 1] == hi))
    {
        return 1;
    }
    return 0;
}




