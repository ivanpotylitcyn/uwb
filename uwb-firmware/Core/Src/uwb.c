#include "uwb.h"
#include "main.h"

#define UWB_RARE_DELAY              10000
#define UWB_DENSE_DELAY             1000

#define SETTINGS_ADDRESS            0x08008800
#define UWB_SUBMERGED_THRESHOLD     25000 // Под водой
#define UWB_ENMERGED_THRESHOLD      20000 // Всплыли
#define BITRATE_RS485               9600
#define LEDMASK                     0xAAAAAAAAAAAAAAAA
#define UWB_BLINK_DELAY             500


uwb_context_t uwb;

//static uint32_t start_rare_moment   = 0;
//static uint32_t start_dense_moment  = 0;
static uint32_t start_blink_moment  = 0;

static uint8_t cnt_mask = 0;

// Запись во FLASH
void write_flash() {

    HAL_FLASH_Unlock(); // Открыть доступ к FLASH (она закрыта от случайной записи)
    uint32_t *source_adr = (void *)&uwb ;

    GenCRC16(source_adr, 20);

    FLASH_EraseInitTypeDef ef; // Объявляю структуру, необходимую для функции стирания страницы
    HAL_StatusTypeDef stat;
    ef.TypeErase = FLASH_TYPEERASE_PAGES; // Стирать постранично
    ef.PageAddress = SETTINGS_ADDRESS; // Адрес страницы для стирания
    ef.NbPages = 1; //Число страниц = 1
    uint32_t temp; // Временная переменная для результата стирания (не использую)
    HAL_FLASHEx_Erase(&ef, &temp); // Вызов функции стирания
    // Будьте уверены, что размер структуры настроек кратен 2 байтам
    for (int i = 0; i < 6; i++) { // Запись всех настроек
        stat = HAL_FLASH_Program (FLASH_TYPEPROGRAMDATA_WORD, SETTINGS_ADDRESS + i*4, *(source_adr+i));
        if (stat != HAL_OK) break; // Если что-то пошло не так - выскочить из цикла
    }

    HAL_FLASH_Lock(); // Закрыть флешку от случайной записи
}

void read_flash() {
    uint32_t *source_adr = (uint32_t *)(SETTINGS_ADDRESS);
    uint32_t *dest_adr = (void *)&uwb;

    if (CheckCRC16(source_adr, 22)) {
        for (uint16_t i=0; i < 6; ++i) {                                  // В цикле производим чтение
                *(dest_adr + i) = *(__IO uint32_t*)(source_adr + i);      // Само чтение
        }
        uwb.bitrate_rs485 = BITRATE_RS485;
        uwb.press_rtig1 = UWB_SUBMERGED_THRESHOLD;
        uwb.press_rtig2 = UWB_ENMERGED_THRESHOLD;
        uwb.led_mask = LEDMASK;
        uwb.ledrate = UWB_BLINK_DELAY;
    } else {
        uwb.bitrate_rs485 = BITRATE_RS485;
        uwb.press_rtig1 = UWB_SUBMERGED_THRESHOLD;
        uwb.press_rtig2 = UWB_ENMERGED_THRESHOLD;
        uwb.led_mask = LEDMASK;
        uwb.ledrate = UWB_BLINK_DELAY;
    }
}

void TM_CRC_INIT() {
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
    CRC->CR |= CRC_CR_POLYSIZE_0;
    CRC->POL = (uint32_t)0x8005;
    CRC->CR |= (CRC_CR_REV_IN_0);
    CRC->CR |= (CRC_CR_REV_OUT);
}

void switch_RS485() {
    USART1 -> CR1 &= ~(USART_CR1_UE);
    USART1 -> BRR = UART_DIV_SAMPLING16(REF_CLK, uwb.bitrate_rs485);
    USART1 -> CR1 |= USART_CR1_UE;

    TIM6->CR1 &= ~(TIM_CR1_CEN);
    TIM6->ARR = (uint32_t)((30000000 / uwb.bitrate_rs485));
    TIM6->CR1 |= TIM_CR1_CEN;
}

void uwb_init()
{

    TM_CRC_INIT();
    read_flash();
    switch_RS485();

    uwb.led_toggle = 0;
    uwb.led_blink = 0;

    uwb.mode = UWB_MODE_COMMAND;
    uwb.state = UWB_ONBOARD;

    // ****************************************
    // Enable BQ
    // ****************************************

    //HAL_GPIO_WritePin(EN_BQ_GPIO_Port, EN_BQ_Pin, GPIO_PIN_SET);		// Enable BQ
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
    ADS122_init();
    HAL_Delay(10);
}

void sensors_handle()
{
    //bme280_read(&uwb.bme280);
    //ps_read(&uwb.ps);
    uwb.water_sink = !HAL_GPIO_ReadPin(water_sens_GPIO_Port, water_sens_Pin);

    if (uwb.state == UWB_ONBOARD && uwb.ps.pressure > uwb.press_rtig1) {
        uwb.mode = UWB_MODE_EMERGENCY;
        //start_dense_moment = HAL_GetTick();
    	uwb.state = UWB_SUBMERGED;
    }

    if (uwb.state == UWB_SUBMERGED && uwb.ps.pressure < uwb.press_rtig1) {
        uwb.state = UWB_ENMERGED;
        cnt_mask = 0;
        uwb.led_toggle = 0;
        uwb.led_blink = 0;
    }

    if (uwb.state == UWB_ENMERGED && uwb.ps.pressure > uwb.press_rtig2) {
        uwb.state = UWB_SUBMERGED;
        uwb.mode = UWB_MODE_EMERGENCY;
        HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, GPIO_PIN_RESET);
        //start_dense_moment = HAL_GetTick();
        cnt_mask = 0;
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
//        if (uwb.state == UWB_ONBOARD && HAL_GetTick() - start_rare_moment > UWB_RARE_DELAY) {
//            start_rare_moment = HAL_GetTick();
//        }
//
//        if (uwb.state == UWB_SUBMERGED && HAL_GetTick() - start_dense_moment > UWB_DENSE_DELAY) {
//            start_dense_moment = HAL_GetTick();
//        }

        if (uwb.state == UWB_ENMERGED && HAL_GetTick() - start_blink_moment > uwb.ledrate) {
            HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, (((uint64_t)0x01 << (cnt_mask & 0x3F)) & uwb.led_mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            cnt_mask++;
            start_blink_moment = HAL_GetTick();
        }

        sensors_handle();
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
