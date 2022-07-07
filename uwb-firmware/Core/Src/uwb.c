#include "uwb.h"
#include "main.h"

#define UWB_STARTUP_DELAY           5000

#define UWB_RARE_DELAY              10000
#define UWB_DENSE_DELAY             1000
#define UWB_BLINK_DELAY             500

#define UWB_SUBMERGED_THRESHOLD     500
#define UWB_ENMERGED_THRESHOLD      8000

static uwb_context_t uwb;

static uint32_t startup_moment      = 0;
static uint32_t start_rare_moment   = 0;
static uint32_t start_dense_moment  = 0;
static uint32_t start_blink_moment  = 0;

void uwb_init()
{
    uwb.mode = UWB_MODE_STARTUP;
    uwb.state = UWB_ONBOARD;

    startup_moment = HAL_GetTick();


    // ****************************************
    // Enable BQ
    // ****************************************

    HAL_GPIO_WritePin(EN_BQ_GPIO_Port, EN_BQ_Pin, GPIO_PIN_SET);		// Enable BQ
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
    // Enable sensors
    // ****************************************

    HAL_GPIO_WritePin(EN_Hall_GPIO_Port, EN_Hall_Pin, GPIO_PIN_RESET);  // Enable Hall sensor
    bme280_init();														// Initialize BME280
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

        HAL_SuspendTick();                                                      // Disable tick interrupt
        HAL_PWR_EnableSleepOnExit();                                            // Enable Sleep mode
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);       // Enter Sleep mode


        // ****************************************
        // Waiting for interrupt to disable sleep
        // ****************************************


        HAL_ResumeTick();                                                       // Enable tick interrupt

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

void modbus_init() { }
void modbus_handle() { }
