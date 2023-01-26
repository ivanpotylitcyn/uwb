#include "uwb.h"
#include "main.h"

uwb_context_t uwb;

//static uint32_t start_rare_moment   = 0;
//static uint32_t start_dense_moment  = 0;
static uint32_t start_blink_moment  = 0;

static uint8_t cnt_mask = 0;

void write_flash() {
    HAL_FLASH_Unlock();                   // Открыть доступ к FLASH (она закрыта от случайной записи)
    uint32_t* source_adr = (void*)&uwb;

    GenCRC16(source_adr, 22);

    FLASH_EraseInitTypeDef ef;            // Объявляю структуру, необходимую для функции стирания страницы
    ef.TypeErase = FLASH_TYPEERASE_PAGES; // Стирать постранично
    ef.PageAddress = SETTINGS_ADDRESS;    // Адрес страницы для стирания
    ef.NbPages = 1;                       // Число страниц = 1
    uint32_t temp;                        // Временная переменная для результата стирания (не использую)
    HAL_FLASHEx_Erase(&ef, &temp);        // Вызов функции стирания

    // Будьте уверены, что размер структуры настроек кратен 2 байтам
    HAL_StatusTypeDef stat;
    for (int i = 0; i < 6; i++) {         // Запись всех настроек
        stat = HAL_FLASH_Program(FLASH_TYPEPROGRAMDATA_WORD, SETTINGS_ADDRESS + i * 4, *(source_adr + i));
        if (stat != HAL_OK) break;        // Если что-то пошло не так - выскочить из цикла
    }

    HAL_FLASH_Lock();                     // Закрыть флешку от случайной записи
}

void read_flash() {
    uint32_t *source_adr = (uint32_t *)(SETTINGS_ADDRESS);
    uint32_t *dest_adr = (void *)&uwb;

    if (CheckCRC16(source_adr, 24)) {
        for (uint16_t i = 0; i < 6; ++i) {                           // В цикле производим чтение
                *(dest_adr + i) = *(__IO uint32_t*)(source_adr + i); // Само чтение
        }
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
    TIM6->ARR = ((13500000 / uwb.bitrate_rs485));
    TIM6->CR1 |= TIM_CR1_CEN;
}

typedef enum {
	LED_RED = 0,
	LED_GREEN,
	LED_BLUE,
	LED_ALL,
} colour_led_t;

static void enable_led(colour_led_t led, bool on) {
	uint16_t pin = 0;

	switch (led) {
		case LED_RED:   pin = LED_R_Pin; break;
		case LED_GREEN: pin = LED_G_Pin; break;
		case LED_BLUE:  pin = LED_B_Pin; break;
		case LED_ALL:	pin = LED_R_Pin | LED_G_Pin | LED_B_Pin; break;
		default: return;
	}

	HAL_GPIO_WritePin(GPIOB, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
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

    for (int i = 0; i < 3; i++) {
		enable_led(LED_ALL, true);
		HAL_Delay(100);
		enable_led(LED_ALL, false);
		HAL_Delay(100);
    }

    // ****************************************
    // Init BQ state
    // ****************************************

    HAL_GPIO_WritePin(EN_BQ_GPIO_Port, EN_BQ_Pin, GPIO_PIN_SET);        // Enable BQ

    uwb.bq.i2c_connected = false;
    uwb.bq.ac_is_present = false;
    uwb.bq.acok = false;

    uwb.bq.charge_current = 1024;
    uwb.bq.charge_voltage = 4096;
    uwb.bq.input_current = 1024;

    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);


    // ****************************************
    // Enable power
    // ****************************************

    HAL_GPIO_WritePin(EN_6V0_GPIO_Port, EN_6V0_Pin, GPIO_PIN_SET);      // Enable 6V0
    HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);      // Enable 3V3
    HAL_GPIO_WritePin(EN_5V0_GPIO_Port, EN_5V0_Pin, GPIO_PIN_SET);      // Enable 5V0
    HAL_GPIO_WritePin(EN_12LED_GPIO_Port, EN_12LED_Pin, GPIO_PIN_SET);  // Enable 12V0

    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);  // Activate RS485 RX
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);  // Activate RS485 RX
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);  // Activate RS485 RX


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
    bme280_init();                                                      // Initialize BME280
    ADS122_init();
    HAL_Delay(10);
}

void sensors_handle()
{
    bme280_read(&uwb.bme280);
    ps_read(&uwb.ps);
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

static uint32_t bq_last_handle_moment = 0;

typedef struct {
	float voltage;
	int percent;
} ChargeTable;

static ChargeTable charge_table[] = {
	{2.900000000, 0},
	{2.989987916, 1},
	{2.999987916, 2},
	{3.014796540, 13},
	{3.029484326, 22},
	{3.043169162, 27},
	{3.063796107, 33},
	{3.095576344, 38},
	{3.144370487, 42},
	{3.234563525, 47},
	{3.296522904, 50},
	{3.364385189, 51},
	{3.377357085, 52},
	{3.415541704, 52},
	{3.454245924, 54},
	{3.486008036, 55},
	{3.522307592, 56},
	{3.577530285, 58},
	{3.722462666, 64},
	{3.846556637, 70},
	{3.993446585, 78},
	{4.144946479, 89},
	{4.170207538, 91},
	{4.182992135, 80},
	{4.199824786, 90},
	{4.200000000, 100}
};

static int charge_table_size = sizeof(charge_table) / sizeof(charge_table[0]);

int find_charge_percent(float voltage) {
	for (int i = 0; i < charge_table_size; i++)
		if (voltage < charge_table[i].voltage)
			return charge_table[i].percent;

	return 100;
}

void charge_handle()
{
    if (HAL_GetTick() - bq_last_handle_moment < 1000)
        return;

    bq_last_handle_moment = HAL_GetTick();


    // ****************************************
    // Battery Voltage ADC control
    // ****************************************

    HAL_ADC_Start(&hadc);							// запускаем преобразование сигнала АЦП
    HAL_ADC_PollForConversion(&hadc, 100);			// ожидаем окончания преобразования
    uint32_t adc_iout = HAL_ADC_GetValue(&hadc);	// читаем полученное значение в переменную adc

    HAL_ADC_Start(&hadc);							// запускаем преобразование сигнала АЦП
    HAL_ADC_PollForConversion(&hadc, 100);			// ожидаем окончания преобразования
    uint32_t adc_bat  = HAL_ADC_GetValue(&hadc);	// читаем полученное значение в переменную adc

    float v_ref = 3.3;
    uint32_t adc_size = 4096;

	uwb.iout_mv = (uint16_t)(v_ref / (float)adc_size * (float)adc_iout * 20 * 1000);	// 20 - коэффициент из документации BQ, 1000 V -> mV

	float volt_bat  = v_ref / (float)adc_size * (float)adc_bat * 1.585;	// 1.58 - коэффициент делителя, припаянного на плате
	uwb.power_percent = find_charge_percent(volt_bat);


    // ****************************************
    // Check UWB mode
    // ****************************************

    if (uwb.mode == UWB_MODE_EMERGENCY) {
        HAL_GPIO_WritePin(EN_BQ_GPIO_Port, EN_BQ_Pin, GPIO_PIN_RESET);        // Disable BQ
        return;
    }

    // ****************************************
    // Check for I2C connection
    // ****************************************

    uwb.bq.i2c_connected = bq24735_is_connected();

    if (!uwb.bq.i2c_connected) {
        uwb.bq.i2c_connected = false;
        uwb.bq.ac_is_present = false;
        enable_led(LED_GREEN, false);
        enable_led(LED_BLUE, false);
        return;
    }

    enable_led(LED_BLUE, true);


    // ****************************************
    // Check AC
    // ****************************************

    uwb.bq.ac_is_present = bq24735_ac_is_present();

    uwb.bq.acok = HAL_GPIO_ReadPin(ACOK_bat_GPIO_Port, ACOK_bat_Pin);

    if (!uwb.bq.ac_is_present || !uwb.bq.acok) {
        enable_led(LED_GREEN, false);
        return;
    }

    enable_led(LED_GREEN, true);


    // ****************************************
    // Enable charging
    // ****************************************

    bq24735_enable_charging(&uwb.bq);

    // Read back config

    uwb.charge_current = bq24735_read_charge_current();
    uwb.charge_voltage = bq24735_read_charge_voltage();
    uwb.input_current  = bq24735_read_input_current();
    uwb.charge_option  = bq24735_read_charge_option();
}

static uint32_t period_ms = 2000;
static uint32_t cycle_ms = 250;
static uint32_t cycle_led_ms = 150;

static void battery_led_handle() {
	if (uwb.power_percent >= 75)
		return;

	uint32_t period_uptime = HAL_GetTick() % period_ms;
	uint32_t cycle_uptime = period_uptime % cycle_ms;
	uint32_t cycle_number = period_uptime / 250;

	if (cycle_number == 0)
		if (cycle_uptime < cycle_led_ms)
			enable_led(LED_RED, true);
		else
			enable_led(LED_RED, false);

	if (cycle_number == 1 && uwb.power_percent < 50)
		if (cycle_uptime < cycle_led_ms)
			enable_led(LED_RED, true);
		else
			enable_led(LED_RED, false);

	if (cycle_number == 2 && uwb.power_percent < 25)
		if (cycle_uptime < cycle_led_ms)
			enable_led(LED_RED, true);
		else
			enable_led(LED_RED, false);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == WAKE_OPTO_Pin)
    HAL_PWR_DisableSleepOnExit();
}

void uwb_handle()
{
    // ****************************************
    // Handle LED
    // ****************************************

    if (uwb.state != UWB_ENMERGED && (bool)uwb.led_blink && HAL_GetTick() - start_blink_moment > uwb.ledrate) {
        HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, (((uint64_t)0x01 << (cnt_mask & 0x3F)) & uwb.led_mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        cnt_mask++;
    	start_blink_moment = HAL_GetTick();
    }


    // ****************************************
    // Handle Hall sensor (go to sleep)
    // ****************************************

    if (!HAL_GPIO_ReadPin(go_to_sleep_GPIO_Port, go_to_sleep_Pin))
        uwb.mode = UWB_MODE_SLEEP;

    GPIO_PinState wakeup_state = HAL_GPIO_ReadPin(WAKE_OPTO_GPIO_Port, WAKE_OPTO_Pin);

    if (wakeup_state)
    	uwb.mode = UWB_MODE_COMMAND;


    // ****************************************
    // Handle UWB state
    // ****************************************

    switch (uwb.mode)
    {
    case UWB_MODE_COMMAND:
        charge_handle();
        sensors_handle();
        battery_led_handle();
        break;

    case UWB_MODE_EMERGENCY:
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
        // HAL_GPIO_WritePin(EN_Hall_GPIO_Port, EN_Hall_Pin, GPIO_PIN_RESET);   // Disable Hall sensor
        enable_led(LED_ALL, false);

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

bool uwb_enable_led(uint16_t enable)
{
    if ((bool)uwb.led_blink || uwb.state == UWB_ENMERGED) {
        return 0;
    }

    uwb.led_toggle = enable;
    HAL_GPIO_WritePin(light_LED_GPIO_Port, light_LED_Pin, (bool)enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return 1;
}

bool uwb_enable_led_blink(uint16_t enable)
{
    if ((bool)uwb.led_toggle || uwb.state == UWB_ENMERGED) {
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
