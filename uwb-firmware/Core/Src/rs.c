#include "main.h"
#include "rs.h"

void rs_init()
{
    HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_RS_GPIO_Port, EN_RS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);
}