#include "uwb.h"

#include "usart.h"
#include "tim.h"

#define MODBUS_READ             0x03
#define MODBUS_WRITE            0x06
#define MODBUS_WRITE_MULTIPLE   0x10
#define ModBusRegsCnt           30

static volatile bool transmiting = false;

static uint8_t str;
static uint8_t buff_uart[255];
static uint8_t cnt = 0;
static uint16_t ping_counter = 0;

uint8_t GenCRC16(uint8_t* arr, uint32_t count);
uint8_t CheckCRC16(uint8_t* arr, uint32_t count);

uint16_t *ModBusRegs[ModBusRegsCnt] = {
                                        (uint16_t *)&(uwb.ping),
                                        (uint16_t *)&(uwb.bme280.temperature),
                                        (uint16_t *)&(uwb.bme280.humidity),
                                        (uint16_t *)&(uwb.bme280.pressure),
                                        (uint16_t *)&(uwb.ps.pressure)+1, (uint16_t *)&(uwb.ps.pressure),
                                        (uint16_t *)&(uwb.press_rtig1)+1,(uint16_t *)&(uwb.press_rtig1),
                                        (uint16_t *)&(uwb.press_rtig2)+1, (uint16_t *)&(uwb.press_rtig2),
                                        (uint16_t *)&(uwb.bitrate_rs485)+1, ((uint16_t *)&(uwb.bitrate_rs485)),
                                        (uint16_t *)&(uwb.led_mask)+3, (uint16_t *)&(uwb.led_mask)+2, (uint16_t *)&(uwb.led_mask)+1, (uint16_t *)&(uwb.led_mask),
                                        (uint16_t *)&(uwb.ledrate),
                                        (uint16_t *)&(uwb.led_toggle),
                                        (uint16_t *)&(uwb.led_blink),
                                        (uint16_t *)&(uwb.water_sink),
                                        (uint16_t *)&(uwb.charge_current),
                                        (uint16_t *)&(uwb.charge_voltage),
                                        (uint16_t *)&(uwb.input_current),
                                        (uint16_t *)&(uwb.charge_option),
										(uint16_t *)&(uwb.power_percent),
										(uint16_t *)&(uwb.iout_mv),
        };

void modbus_init()
{
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Enable receiver / Disable transmitter

    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
    HAL_UART_Receive_IT(&huart1, &str, 1);
    //HAL_UART_Receive_DMA(&huart1, &str, 1);
}

void rs485_transmit(uint8_t* buff_uart, uint16_t cnt)
{
	transmiting = true;

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_SET); // Activate RS485 TX
    //HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);
    HAL_UART_Transmit_DMA(&huart1, buff_uart, cnt);
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
    //HAL_UART_Receive_DMA(&huart1, &str, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
        return;

    cnt = 0;
    
    if (USART1 -> BRR != UART_DIV_SAMPLING16(REF_CLK, uwb.bitrate_rs485)) {
        switch_RS485();
    }

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Activate RS485 RX
    HAL_UART_Receive_IT(&huart1, &str, 1);
    //HAL_UART_Receive_DMA(&huart1, &str, 1);
    transmiting = false;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) // Проверка завершения транзакции по modbus
        return;

    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SetCounter(&htim6, 0); // сброс таймера

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

    uwb_modbus_register_t register_address = (((uint16_t)buff_uart[2]) << 8) | ((uint16_t)buff_uart[3]);
    uint16_t num_word = (((uint16_t)buff_uart[4]) << 8) | ((uint16_t)buff_uart[5]);

    bool success = 1;

    switch (buff_uart[1])
    {
    case MODBUS_READ:
        if ((register_address + num_word) <= ModBusRegsCnt) {
            uint8_t ModBusTX_Cnt = 2;
            buff_uart[ModBusTX_Cnt++] = (uint8_t)(num_word * 2);
            
            if ((register_address) == 0)
              uwb.ping = ping_counter++;

            for (int i = 0; i < num_word; i++) {
                uint16_t Val = (uint16_t)*ModBusRegs[register_address + i];
                buff_uart[ModBusTX_Cnt++] = (Val >> 8) & 0xFF;
                buff_uart[ModBusTX_Cnt++] = Val & 0xFF;
            }

            cnt = GenCRC16(buff_uart, buff_uart[2] + 3);
        }

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

            case UWB_MASK_LED_H1:
                uwb.led_mask &= ~(((uint64_t)0xFFFF) << 48);
                uwb.led_mask |= ((uint64_t)num_word) << 48;

                break;

            case UWB_MASK_LED_L1:
                uwb.led_mask &= ~(((uint64_t)0xFFFF) << 32);
                uwb.led_mask |= ((uint64_t)num_word) << 32;

                break;

            case UWB_MASK_LED_H0:
                uwb.led_mask &= ~(((uint64_t)0xFFFF) << 16);
                uwb.led_mask |= ((uint64_t)num_word) << 16;

                break;

            case UWB_MASK_LED_L0:
                uwb.led_mask &= ~((uint64_t)0xFFFF);
                uwb.led_mask |= (uint64_t)num_word;

                break;

            case UWB_CHARGE_CURRENT:
                uwb.bq.charge_current = num_word;
                charge_handle();
                break;

            case UWB_CHARGE_VOLTAGE:
                uwb.bq.charge_voltage = num_word;
                charge_handle();
                break;

            case UWB_INPUT_CURRENT:
                uwb.bq.input_current = num_word;
                charge_handle();
                break;

            case UWB_CHARGE_OPTION:
            	if (uwb.bq.i2c_connected)
            		bq24735_write_charge_option(num_word);

                charge_handle();
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
                    uwb_enable_led_blink(0);
                }
                break;

            case UWB_SAVE_FLSH:
                if (num_word) {
                    write_flash();
                }
                break;

            case UWB_SET_DEFAULTS:
            	if (num_word) {
                    uwb.bitrate_rs485 = BITRATE_RS485;
                    uwb.press_rtig1 = UWB_SUBMERGED_THRESHOLD;
                    uwb.press_rtig2 = UWB_ENMERGED_THRESHOLD;
                    uwb.led_mask = LEDMASK;
                    uwb.ledrate = UWB_BLINK_DELAY;

                    uwb.bq.charge_current = 1024;
                    uwb.bq.charge_voltage = 4096;
                    uwb.bq.input_current = 1024;
            	}

            default:
                //ошибка
                break;
        }

        if (!success) {
            buff_uart[1] |= (buff_uart[1] | 0x80);
            buff_uart[2] = 0x01;
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

            case UWB_PRESS_TRIG_1_H:

                uwb.press_rtig1 = ((uint32_t)buff_uart[7] << 24) | ((uint32_t)buff_uart[8] << 16);
                uwb.press_rtig1 |= (((uint32_t)buff_uart[9] << 8)) | (uint32_t)buff_uart[10];
                break;

            case UWB_PRESS_TRIG_2_H:

                uwb.press_rtig2 = ((uint32_t)buff_uart[7] << 24) | ((uint32_t)buff_uart[8] << 16);
                uwb.press_rtig2 |= (((uint32_t)buff_uart[9] << 8)) | (uint32_t)buff_uart[10];
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

uint8_t GenCRC16(uint8_t* arr, uint32_t count)
{
    CRC->CR |= CRC_CR_RESET;

    uint32_t cnt;

    /* Calculate number of 32-bit blocks */
    cnt = count >> 2;

    /* Calculate */
    while (cnt--) {
        /* Set new value */
        CRC->DR = arr[3] + (arr[2] << 8) + (arr[1] << 16) + (arr[0] << 24);

        /* Increase by 4 */
        arr += 4;
    }

    /* Calculate remaining data as 8-bit */
    cnt = count % 4;

    /* Calculate */
    while (cnt--) {
        /* Set new value */
        *((uint8_t *)&CRC->DR) = *arr++;
    }

    *arr++ = (uint8_t)CRC->DR;
    *arr++ = (uint8_t)(CRC->DR >> 8);

    return count + 2;
}

uint8_t CheckCRC16(uint8_t* arr, uint32_t count)
{

    CRC->CR |= CRC_CR_RESET;

    uint32_t cnt;

    /* Calculate number of 32-bit blocks */
    cnt = count >> 2;

    /* Calculate */
    while (cnt--) {
        /* Set new value */
        CRC->DR = arr[3] + (arr[2] << 8) + (arr[1] << 16) + (arr[0] << 24);

        /* Increase by 4 */
        arr += 4;
    }

    /* Calculate remaining data as 8-bit */
    cnt = count % 4;

    /* Calculate */
    while (cnt--) {
        /* Set new value */
        *((uint8_t *)&CRC->DR) = *arr++;
    }

    if (CRC->DR) return 0;
    else return 1;
}


//for (uint16_t i = 0; i < num_word; i++)
//                {
//                    switch (register_address)
//                    {
//                      case UWB_PING:
//                          uwb.ping = ping_counter;
//                          ping_counter++;
//                          break;
//
//                        case UWB_TEMPERATURE:
//                            register_value = (uint16_t)uwb.bme280.temperature;
//                            break;
//
//                        case UWB_HUMIDITY:
//                            register_value = (uint16_t)uwb.bme280.humidity;
//                            break;
//
//                        case UWB_PRESSURE:
//                            register_value = (uint16_t)uwb.bme280.pressure;
//                            break;
//
//                        case UWB_PRESSURE_ADS:
//                            register_value = (uint16_t)uwb.ps.pressure;
//                            break;
//
//                        case UWB_BITRATE_RS485_H:
//                            register_value= (uint16_t)((uwb.bitrate_rs485) >> 16);
//                            break;
//
//                        case UWB_BITRATE_RS485_L:
//                            register_value = (uint16_t)uwb.bitrate_rs485;
//                            break;
//
//                        case UWB_MASK_LED_H1:
//                            register_value= (uint16_t)((uwb.led_mask) >> 48);
//                            break;
//
//                        case UWB_MASK_LED_L1:
//                            register_value = (uint16_t)((uwb.led_mask) >> 32);
//                            break;
//
//                        case UWB_MASK_LED_H0:
//                            register_value= (uint16_t)((uwb.led_mask) >> 16);
//                            break;
//
//                        case UWB_MASK_LED_L0:
//
//                            register_value = (uint16_t)uwb.led_mask;
//                            break;
//
//                        case UWB_LEDRATE:
//                            register_value = (uint16_t)uwb.ledrate;
//                            break;
//
//                        case UWB_PRESS_TRIG_1:
//                            register_value = (uint16_t)uwb.press_rtig1;
//                            break;
//
//                        case UWB_PRESS_TRIG_2:
//                            register_value = (uint16_t)uwb.press_rtig2;
//                            break;
//
//                        case UWB_LED_TOGGLE:
//                            register_value = (uint16_t)uwb.led_toggle;
//                            break;
//
//                        case UWB_LED_BLINK:
//                            register_value = (uint16_t)uwb.led_blink;
//                            break;
//
//                        default:
//                          register_value = 0xff;
//                            break;
//                    }
//
//                    buff_uart[addr_array + 3] = (uint8_t)(register_value >> 8);
//                    buff_uart[addr_array + 4] = (uint8_t)(register_value);
//
//                    register_address++;
//                    addr_array += 2;
//                }
//
//                buff_uart[2] = num_word * 2;
//
//                cnt = GenCRC16(buff_uart, buff_uart[2] + 3);




