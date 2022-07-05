#include "ps.h"
#include "ads1220.h"

void ps_read(ps_context_t* ps) {
	ps->pressure = ADS1220ReadData();
}
