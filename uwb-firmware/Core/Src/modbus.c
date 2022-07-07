#include "uwb.h"

#include "usart.h"
#include "tim.h"

#define MODBUS_READ     0x03
#define MODBUS_WRITE    0x06

static uint8_t str;
static uint8_t buff_uart[255];
static uint8_t cnt = 0;

uint8_t GenCRC16(uint8_t* buff, size_t len);
uint8_t CheckCRC16(uint8_t* buff, size_t len);

void modbus_init()
{
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
    HAL_UART_Receive_IT(&huart1, &str, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
        return;
    
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SetCounter(&htim6, 0);
    HAL_TIM_Base_Start_IT(&htim6);

    buff_uart[cnt] = str;
    cnt++;

    HAL_UART_Receive_IT(&huart1, &str, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
        return;

    cnt = 0;
    
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Activate RS485 RX
    HAL_UART_Receive_IT(&huart1, &str, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) // Проверка завершения транзакции по modbus
        return;

    if (cnt <= 4) {
        HAL_UART_Receive_IT(&huart1, &str, 1);
        return;
    }

    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SetCounter(&htim6, 0); // сброс таймера

    if (buff_uart[0] != UWB_MODBUS_ID || !CheckCRC16(buff_uart, cnt)) {
        cnt = 0;

        HAL_UART_Receive_IT(&huart1, &str, 1);
        return;
    }

    uwb_modbus_register_t register_address = (buff_uart[2] << 8) |  buff_uart[3];
    uint16_t register_value;

    uint16_t num_word = (buff_uart[4] << 8) |  buff_uart[5];
    uint8_t addr_array = 0;

    buff_uart[2] = num_word * 2;

    switch (buff_uart[1])
    {
    case MODBUS_READ:
        for (uint16_t i = 0; i < num_word; i++)
        {
            switch (register_address)
            {
                case UWB_TEMPERATURE:
                    register_value = (uint16_t)uwb.bme280.temperature;
                    break;

                case UWB_HUMIDITY:
                    register_value = (uint16_t)uwb.bme280.humidity;
                    break;

                case UWB_PRESSURE:
                    register_value = (uint16_t)uwb.bme280.pressure;
                    break;

                default:
                    //ошибка
                    break;
            }

            buff_uart[addr_array + 3] = (uint8_t)(register_value >> 8);
            buff_uart[addr_array + 4] = (uint8_t)(register_value);

            register_address++;
            addr_array += 2;
        }

        cnt = GenCRC16(buff_uart, buff_uart[2] + 3);

        break;

    case MODBUS_WRITE:
        //запись в в регистр
        break;
    
    default:
        break;
    }

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_SET); // Activate RS485 TX
    HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);
}

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
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }

    lo = crc & 0xFF;
    hi = (crc >> 8) & 0xFF;

    buff[len++] = lo;
    buff[len++] = hi;

    return len;
}

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
            else {
                crc >>= 1;
            }
        }
    }

    lo = crc & 0xFF;
    hi = (crc >> 8) & 0xFF;

    if ((buff[len - 2] == lo) && (buff[len - 1] == hi))
        return 1;

    return 0;
}
