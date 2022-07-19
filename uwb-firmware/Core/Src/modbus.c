#include "uwb.h"

#include "usart.h"
#include "tim.h"

#define MODBUS_READ             0x03
#define MODBUS_WRITE            0x06
#define MODBUS_WRITE_MULTIPLE   0x10

static volatile bool transmiting = false;

static uint8_t str;
static uint8_t buff_uart[255];
static uint8_t cnt = 0;

static uint16_t ping_counter = 0;

uint8_t GenCRC16(uint8_t* buff, size_t len);
uint8_t CheckCRC16(uint8_t* buff, size_t len);

void modbus_init()
{
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Enable receiver / Disable transmitter

    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
    HAL_UART_Receive_IT(&huart1, &str, 1);
}

void rs485_transmit(uint8_t* buff_uart, uint16_t cnt)
{
	transmiting = true;

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_SET); // Activate RS485 TX
    HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
        return;
    
    if (transmiting)
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
    
    if (USART1 -> BRR != UART_DIV_SAMPLING16(REF_CLK, uwb.bitrate_rs485)) {
        USART1 -> CR1 &= ~(USART_CR1_UE);
        USART1 -> BRR = UART_DIV_SAMPLING16(REF_CLK, uwb.bitrate_rs485);
        USART1 -> CR1 |= USART_CR1_UE;

        TIM6->CR1 &= ~(TIM_CR1_CEN);
        TIM6->ARR = (uint32_t)((30000000 / uwb.bitrate_rs485));
        TIM6->CR1 |= TIM_CR1_CEN;
    }

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Activate RS485 RX
    HAL_UART_Receive_IT(&huart1, &str, 1);
    transmiting = false;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SetCounter(&htim6, 0); // сброс таймера

    if (htim->Instance != TIM6) // Проверка завершения транзакции по modbus
        return;

    if (cnt <= 4) {
        cnt = 0;

//        HAL_UART_Receive_IT(&huart1, &str, 1);
        return;
    }


    if (buff_uart[0] != UWB_MODBUS_ID || !CheckCRC16(buff_uart, cnt)) {
        cnt = 0;

//        HAL_UART_Receive_IT(&huart1, &str, 1);
        return;
    }

    uwb_modbus_register_t register_address = (buff_uart[2] << 8) |  buff_uart[3];
    uint16_t register_value;

    uint16_t num_word = (buff_uart[4] << 8) |  buff_uart[5];
    uint8_t addr_array = 0;

    bool success = 1;

    switch (buff_uart[1])
    {
    case MODBUS_READ:
        for (uint16_t i = 0; i < num_word; i++)
        {
            switch (register_address)
            {
				case UWB_PING:
					register_value = ping_counter;
					ping_counter++;
					break;

                case UWB_TEMPERATURE:
                    register_value = (uint16_t)uwb.bme280.temperature;
                    break;

                case UWB_HUMIDITY:
                    register_value = (uint16_t)uwb.bme280.humidity;
                    break;

                case UWB_PRESSURE:
                    register_value = (uint16_t)uwb.bme280.pressure;
                    break;

                case UWB_PRESSURE_ADS:
                    register_value = (uint16_t)uwb.ps.pressure;
                    break;

                case UWB_BITRATE_RS485_H:
                    register_value= (uint16_t)((uwb.bitrate_rs485) >> 16);
                    break;

                case UWB_BITRATE_RS485_L:
                    register_value = (uint16_t)uwb.bitrate_rs485;
                    break;

                case UWB_MASK_LED_H1:
                    register_value= (uint16_t)((uwb.led_mask) >> 48);
                    break;

                case UWB_MASK_LED_L1:
                    register_value = (uint16_t)((uwb.led_mask) >> 32);
                    break;

                case UWB_MASK_LED_H0:
                    register_value= (uint16_t)((uwb.led_mask) >> 16);
                    break;

                case UWB_MASK_LED_L0:

                    register_value = (uint16_t)uwb.led_mask;
                    break;

                case UWB_LEDRATE:
                    register_value = (uint16_t)uwb.ledrate;
                    break;

                case UWB_PRESS_TRIG_1:
                    register_value = (uint16_t)uwb.press_rtig1;
                    break;

                case UWB_PRESS_TRIG_2:
                    register_value = (uint16_t)uwb.press_rtig2;
                    break;

                case UWB_LED_TOGGLE:
                    register_value = (uint16_t)uwb.led_toggle;
                    break;

                case UWB_LED_BLINK:
                    register_value = (uint16_t)uwb.led_blink;
                    break;

                case UWB_RESET:
                    register_value = 0;
                    break;

                case UWB_RESTART:
                    register_value = 0;
                    break;

                default:
                	register_value = 0xff;
                    break;
            }

            buff_uart[addr_array + 3] = (uint8_t)(register_value >> 8);
            buff_uart[addr_array + 4] = (uint8_t)(register_value);

            register_address++;
            addr_array += 2;
        }

        buff_uart[2] = num_word * 2;

        cnt = GenCRC16(buff_uart, buff_uart[2] + 3);

        break;

    case MODBUS_WRITE:
        switch (register_address)
        {
            case UWB_LED_TOGGLE:
                success = uwb_enable_led(num_word);
                break;

            case UWB_LED_BLINK:
                success = uwb_enable_led_blink(num_word);
            	break;

            case UWB_LEDRATE:
                uwb.ledrate = num_word;
                break;

            case UWB_PRESS_TRIG_1:
                uwb.press_rtig1 = num_word;
                break;

            case UWB_PRESS_TRIG_2:
                uwb.press_rtig2 = num_word;
                break;

            case UWB_RESET:
                if (num_word) {
                    NVIC_SystemReset();
                }
                break;

            case UWB_RESTART:
                if (num_word) {
                    uwb.mode = UWB_MODE_COMMAND;
                    uwb.state = UWB_ONBOARD;
                    uwb_enable_led(0);
                }
                break;

            default:
                //ошибка
                break;
        }

        if (!success) {
            buff_uart[1] |= (buff_uart[1] | 0x80);
            buff_uart[1] = 0x01;
            cnt = GenCRC16(buff_uart, 3);
        }

        break;

    case MODBUS_WRITE_MULTIPLE:
        switch (register_address)
        {
            case UWB_BITRATE_RS485_H:

                uwb.bitrate_rs485 = ((uint32_t)buff_uart[7] << 24) | ((uint32_t)buff_uart[8] << 16);
                uwb.bitrate_rs485 |= (((uint32_t)buff_uart[9] << 8)) | (uint32_t)buff_uart[10];
                break;

            case UWB_MASK_LED_H1:

                uwb.led_mask = ((uint64_t)buff_uart[7] << 56) | ((uint64_t)buff_uart[8] << 48);
                uwb.led_mask |= (((uint64_t)buff_uart[9] << 40)) | (((uint64_t)buff_uart[10] << 32));
                uwb.led_mask |= ((uint64_t)buff_uart[11] << 24) | ((uint64_t)buff_uart[12] << 16);
                uwb.led_mask |= (((uint64_t)buff_uart[13] << 8)) | (uint64_t)buff_uart[14];
                break;


            default:
                //ошибка
                break;
        }

        cnt = GenCRC16(buff_uart, 6);
        break;

        default:
            break;
    }

    rs485_transmit(buff_uart, cnt);
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
