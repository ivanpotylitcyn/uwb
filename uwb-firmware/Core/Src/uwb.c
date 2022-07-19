#include "uwb.h"
#include "main.h"

#define UWB_RARE_DELAY              10000
#define UWB_DENSE_DELAY             1000
#define UWB_BLINK_DELAY             500

#define UWB_SUBMERGED_THRESHOLD     25000 // Под водой
#define UWB_ENMERGED_THRESHOLD      20000 // Всплыли

#define ID_STM32 0x01

uwb_context_t uwb;

extern UART_HandleTypeDef huart1;

static uint32_t start_rare_moment   = 0;
static uint32_t start_dense_moment  = 0;
static uint32_t start_blink_moment  = 0;

static uint8_t cnt_mask = 0;


void uwb_init()
{
    uwb.mode = UWB_MODE_COMMAND;
    uwb.state = UWB_ONBOARD;

    uwb.bitrate_rs485 = (uwb.bitrate_rs485) ? uwb.bitrate_rs485 : huart1.Init.BaudRate;

    TIM6->ARR = (uint32_t)((30000000 / uwb.bitrate_rs485));

    uwb.ledrate = (uwb.ledrate) ? uwb.ledrate : UWB_BLINK_DELAY;
    uwb.press_rtig1 = (uwb.press_rtig1) ? uwb.press_rtig1 : UWB_SUBMERGED_THRESHOLD;
    uwb.press_rtig2 = (uwb.press_rtig2) ? uwb.bitrate_rs485 : UWB_ENMERGED_THRESHOLD;

    uwb.led_mask = (uwb.led_mask) ? uwb.led_mask : 0xAAAAAAAAAAAAAAAA;

    // ****************************************
    // Enable BQ
    // ****************************************

    HAL_GPIO_WritePin(EN_BQ_GPIO_Port, EN_BQ_Pin, GPIO_PIN_SET);		// Enable BQ
    //bq_init(&uwb.bq);


    // ****************************************
    // Enable power
    // ****************************************

    HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_SET);      // Enable 6V0
    HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);      // Enable 3V3
    HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_SET);      // Enable 5V0
    HAL_GPIO_WritePin(EN_12LED_GPIO_Port, EN_12LED_Pin, GPIO_PIN_SET);  // Enable 12V0


    // ****************************************
    // Enable RS
    // ****************************************

    HAL_GPIO_WritePin(EN_RS_GPIO_Port, EN_RS_Pin, GPIO_PIN_RESET);      // Open P-transistor
    modbus_init();


    // ****************************************
    // Enable sensors
    // ****************************************

    HAL_GPIO_WritePin(EN_Hall_GPIO_Port, EN_Hall_Pin, GPIO_PIN_RESET);  // Enable Hall sensor
    HAL_Delay(10);
    bme280_init();														// Initialize BME280
    ads122_init();
    HAL_Delay(10);
}

void sensors_handle()
{
    bme280_read(&uwb.bme280);
    ps_read(&uwb.ps);
    uwb.water_sink = !HAL_GPIO_ReadPin(water_sens_GPIO_Port, water_sens_Pin);

    if (uwb.state == UWB_ONBOARD && uwb.ps.pressure > uwb.press_rtig1) {
        uwb.mode = UWB_MODE_EMERGENCY;
        start_dense_moment = HAL_GetTick();
    	uwb.state = UWB_SUBMERGED;
    }

    if (uwb.state == UWB_SUBMERGED && uwb.ps.pressure < uwb.press_rtig2) {
        uwb.state = UWB_ENMERGED;
        cnt_mask = 0;
        uwb.led_toggle = 0;
        uwb.led_blink = 0;
    }
}

void uwb_handle()
{
    //bq_handle(&uwb.bq);

    // ****************************************
    // Handle LED
    // ****************************************

    if (uwb.state != UWB_ENMERGED && uwb.led_blink && HAL_GetTick() - start_blink_moment > uwb.ledrate) {
        HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, (((uint64_t)0x01 << (cnt_mask & 0x3F)) & uwb.led_mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        cnt_mask++;
    	start_blink_moment = HAL_GetTick();
    }


    // ****************************************
    // Handle Hall sensor (got to sleep)
    // ****************************************

    if (!HAL_GPIO_ReadPin(go_to_sleep_GPIO_Port, go_to_sleep_Pin))
        uwb.mode = UWB_MODE_SLEEP;


    // ****************************************
    // Handle UWB state
    // ****************************************

    switch (uwb.mode)
    {
    case UWB_MODE_COMMAND:
    	sensors_handle();
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

        if (uwb.state == UWB_ENMERGED && HAL_GetTick() - start_blink_moment > uwb.ledrate) {
            HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, (((uint64_t)0x01 << (cnt_mask & 0x3F)) & uwb.led_mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            cnt_mask++;
            start_blink_moment = HAL_GetTick();

            sensors_handle();
        }
        break;

    case UWB_MODE_SLEEP:
        HAL_GPIO_WritePin(EN_12LED_GPIO_Port, EN_12LED_Pin, GPIO_PIN_RESET);    // Disable 12LED
        HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_RESET);        // Disable 5V0
        HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_RESET);        // Disable 3V3
        HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_RESET);        // Disable 6V0

        HAL_SuspendTick();                                                      // Disable tick interrupt
        HAL_PWR_EnableSleepOnExit();                                            // Enable Sleep mode
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);       // Enter Sleep mode


        // ****************************************
        // Waiting for interrupt to disable sleep
        // ****************************************


        HAL_ResumeTick();                                                       // Enable tick interrupt

        uwb_init();

        break;

    default:
        break;
    }
}

bool uwb_enable_led(bool enable)
{
    if (uwb.led_blink || uwb.state == UWB_ENMERGED) {
        return 0;
    }
    uwb.led_toggle = enable;
	HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

	return 1;
}

bool uwb_enable_led_blink(bool enable)
{
    if (uwb.led_toggle || uwb.state == UWB_ENMERGED) {
        return 0;
    }

    cnt_mask = 0;
    uwb.led_blink = enable;

	if (enable)
		start_blink_moment = HAL_GetTick();
	else
		HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, GPIO_PIN_RESET);

	return 1;
}
