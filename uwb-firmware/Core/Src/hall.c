#include "main.h"
#include "hall.h"

void hall_init()
{
    HAL_GPIO_WritePin(EN_Hall_GPIO_Port, EN_Hall_Pin, GPIO_PIN_RESET);
}