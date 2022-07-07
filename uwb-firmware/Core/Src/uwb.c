#include "uwb.h"
#include "main.h"

#define UWB_STARTUP_DELAY           5000

#define UWB_RARE_DELAY              10000
#define UWB_DENSE_DELAY             1000
#define UWB_BLINK_DELAY             500

#define UWB_SUBMERGED_THRESHOLD     500
#define UWB_ENMERGED_THRESHOLD      8000

#define ID_STM32 0x01

static uwb_context_t uwb;

static uint32_t startup_moment      = 0;
static uint32_t start_rare_moment   = 0;
static uint32_t start_dense_moment  = 0;
static uint32_t start_blink_moment  = 0;

uint8_t str;
uint8_t buff_uart[255];
uint8_t cnt = 0;

void uwb_init()
{
    uwb.mode = UWB_MODE_STARTUP;
    uwb.state = UWB_ONBOARD;

    startup_moment = HAL_GetTick();


    // ****************************************
    // Enable BQ
    // ****************************************

    bq_init(&uwb.bq);


    // ****************************************
    // Enable power
    // ****************************************

    HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_SET);      // Enable 6V0
    HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);      // Enable 3V3
    HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_SET);      // Enable 5V0


    // ****************************************
    // Enable RS
    // ****************************************

    HAL_GPIO_WritePin(EN_RS_GPIO_Port, EN_RS_Pin, GPIO_PIN_RESET);      // Open P-transistor
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);  // Enable receiver / Disable transmitter

    modbus_init();


    // ****************************************
    // Enable Hall sensor
    // ****************************************

    HAL_GPIO_WritePin(EN_Hall_GPIO_Port, EN_Hall_Pin, GPIO_PIN_RESET);  // Enable Hall sensor
}

void sensors_handle()
{
    bme280_read(&uwb.bme280);
    ps_read(&uwb.ps);
    uwb.water_sink = !HAL_GPIO_ReadPin(water_sens_GPIO_Port, water_sens_Pin);

    if (uwb.state == UWB_ONBOARD && uwb.ps.pressure > UWB_SUBMERGED_THRESHOLD)
        uwb.state = UWB_SUBMERGED;

    if (uwb.state == UWB_SUBMERGED && uwb.ps.pressure < UWB_ENMERGED_THRESHOLD) {
        HAL_GPIO_WritePin(EN_12LED_GPIO_Port, EN_12LED_Pin, GPIO_PIN_SET);
        uwb.state = UWB_ENMERGED;
    }
}

void uwb_handle()
{
    // ****************************************
    // Handle Hall sensor
    // ****************************************

    if (!HAL_GPIO_ReadPin(go_to_sleep_GPIO_Port, go_to_sleep_Pin))
        uwb.mode = UWB_MODE_SLEEP;


    // ****************************************
    // Handle UWB state
    // ****************************************

    switch (uwb.mode)
    {
    case UWB_MODE_STARTUP:
        bq_handle(&uwb.bq);
        sensors_handle();
        modbus_handle();

        if (HAL_GetTick() - startup_moment >= UWB_STARTUP_DELAY) {
            HAL_GPIO_WritePin(EN_RS_GPIO_Port, EN_RS_Pin, GPIO_PIN_SET);        // Close P-transistor
            uwb.mode = UWB_MODE_EMERGENCY;
        }
        break;

    case UWB_MODE_COMMAND:
        bq_handle(&uwb.bq);
        sensors_handle();
        modbus_handle();
        break;

    case UWB_MODE_EMERGENCY:
        if (uwb.state == UWB_ONBOARD && HAL_GetTick() - start_rare_moment > UWB_RARE_DELAY) {
            start_rare_moment = HAL_GetTick();

            sensors_handle();
        }

        if (uwb.state == UWB_SUBMERGED && HAL_GetTick() - start_dense_moment > UWB_DENSE_DELAY) {
            start_dense_moment = HAL_GetTick();

            sensors_handle();
        }

        if (uwb.state == UWB_ENMERGED && HAL_GetTick() - start_blink_moment > UWB_BLINK_DELAY) {
            HAL_GPIO_TogglePin(light_LED_GPIO_Port, light_LED_Pin);
            start_blink_moment = HAL_GetTick();

            sensors_handle();
        }
        break;

    case UWB_MODE_SLEEP:
        // Disable power
        HAL_GPIO_WritePin(EN_12LED_GPIO_Port, EN_12LED_Pin, GPIO_PIN_RESET);    // Disable 12LED
        HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_RESET);        // Disable 5V0
        HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_RESET);        // Disable 3V3
        HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_RESET);        // Disable 6V0

        HAL_SuspendTick();                                                      // Disable tick interrrupt
        HAL_PWR_EnableSleepOnExit();                                            // Enable Sleep mode
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);       // Enter Sleep mode


        // ****************************************
        // Waiting for interrupt to disable sleep
        // ****************************************


        HAL_ResumeTick();                                                       // Enable tick interrrupt

        HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_SET);          // Enable 5V0
        HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);          // Enable 3V3
        HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_SET);          // Enable 6V0

        uwb.mode = UWB_MODE_EMERGENCY;
        uwb.state = UWB_ONBOARD;

        break;

    default:
        break;
    }
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
uint8_t CheckCRC16(uint8_t* buff, size_t len) {
    uint16_t crc = 0xFFFF;
    uint16_t pos = 0;
    uint8_t i = 0;
    uint8_t lo = 0;
    uint8_t hi = 0;

    for (pos = 0; pos < len - 2; pos++)
    {
        crc ^= buff[pos];

        for (i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
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
        (buff[len - 1] == hi)) {
        return 1;
    }
    return 0;
}

void modbus_init() {
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
    HAL_UART_Receive_IT(&huart1, &str, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
      //Прием байт по modbus
    if(huart == &huart1) {
        HAL_TIM_Base_Stop_IT(&htim6);
        __HAL_TIM_SetCounter(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);
        buff_uart[cnt] = str;
        cnt = cnt + 1;
        HAL_UART_Receive_IT(&huart1, &str, 1);
    }
  }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
      // проверка окончания транзакция по modbus
    if(htim->Instance == TIM6) {
        uint16_t addr_var = 0;
        uint16_t num_word = 0;
        uint8_t addr_array = 0;

        if (cnt > 4) {
            HAL_TIM_Base_Stop_IT(&htim6);
            __HAL_TIM_SetCounter(&htim6, 0); // сброс таймера

            if (buff_uart[0] == ID_STM32 && CheckCRC16(buff_uart, cnt)) {
                addr_var = (buff_uart[2] << 8) |  buff_uart[3];
                num_word = (buff_uart[4] << 8) |  buff_uart[5];
                buff_uart[2] = num_word * 2;
                if (buff_uart[1] == 0x03) { // чтение
                    for(uint16_t i = 0; i < num_word; i = i + 1) {
                        switch (addr_var) {
                            case 0x0001: // чтение темпы
                                buff_uart[addr_array + 3] = 0x33;
                                buff_uart[addr_array + 4] =  0x44;
                                break;
                            case 0x0002: // еще переменная
                                buff_uart[addr_array + 3] = 0x33;
                                buff_uart[addr_array + 4] = 0x44;
                                break;
                            case 0x0003: // еще переменная
                                buff_uart[addr_array + 3] = 0x33;
                                buff_uart[addr_array + 4] = 0x44;
                                break;
                            default:
                                //ошибка
                              break;
                        }
                        addr_var = addr_var + 1;
                        addr_array = addr_array + 2;
                    }

                    cnt = GenCRC16(buff_uart, buff_uart[2] + 3);

                } else if (buff_uart[1] == 0x06) { // запись
                    //запись в в регистр
                }

            }

            HAL_GPIO_WritePin(GPIOA, UART_DE_Pin, GPIO_PIN_SET); // активация передачи
            HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);
        } else {
            HAL_UART_Receive_IT(&huart1, &str, 1);
        }
        cnt = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart1) {
      HAL_GPIO_WritePin(GPIOA, UART_DE_Pin, GPIO_PIN_RESET);
      HAL_UART_Receive_IT(&huart1, &str, 1);
      cnt = 0;
    }
}

void modbus_handle() { }
