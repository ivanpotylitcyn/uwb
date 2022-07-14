#include "ps.h"
#include "ads1220.h"

void ps_read(ps_context_t* ps) {
    ads_start();
    while(HAL_GPIO_ReadPin(nDRDY_MD_GPIO_Port, nDRDY_MD_Pin) == GPIO_PIN_SET) {}
	ps->pressure = ADS122C04_getConversionData();
}
