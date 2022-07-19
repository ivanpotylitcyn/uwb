#include "ps.h"
#include "ads1220.h"

uint32_t timeout_ads = 0;

void ps_read(ps_context_t* ps) {
    ads_start();
    timeout_ads = HAL_GetTick();
    while((HAL_GPIO_ReadPin(nDRDY_MD_GPIO_Port, nDRDY_MD_Pin) == GPIO_PIN_SET) && HAL_GetTick() - timeout_ads < 100) {}

    if (HAL_GetTick() - timeout_ads < 100) {

        ps->pressure = ADS122C04_getConversionData();
    } else {
        //Датчик давления не ответил

    }

}
