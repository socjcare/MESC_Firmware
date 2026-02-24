#include "stm32fxxx_hal.h"
#include "stdbool.h"



//volatile tle_pkt_t pkt;




bool tle_read_start_dma(void);

static void tle_recover_spi_dma(void);
