#include "main.h"
#include "ws.h"

void ws_init()
{
    HAL_GPIO_WritePin(EN_water_sens_GPIO_Port, EN_water_sens_Pin, GPIO_PIN_SET);
}

void ws_handle()
{
    GPIO_PinState state;
    state = HAL_GPIO_ReadPin(water_sens_GPIO_Port, water_sens_Pin);
}