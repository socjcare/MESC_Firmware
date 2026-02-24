#include "stm32fxxx_hal.h"
#include "stdbool.h"



//volatile tle_pkt_t pkt;
#ifndef TLE5012_DMA_H
#define TLE5012_DMA_H

#include <stdint.h>

typedef enum {
    TLE_IDLE,
    TLE_TX,
    TLE_RX,
    TLE_DONE,
    TLE_ERR
} tle_state_t;

/* Declare variable only */
extern volatile tle_state_t tle_state;

#endif

typedef struct __attribute__((packed)) {
    uint16_t safety;
    uint16_t angle;
    uint16_t speed;
    uint16_t revolutions;
    // add more uint16_t fields if your ND expects more words
} tle_pkt_t;

extern volatile tle_pkt_t pkt;


extern SPI_HandleTypeDef hspi3;

bool tle_read_start_dma(void);
//volatile uint16_t tle_angle_latest = 0;   // 0..65535 mapped
//volatile uint8_t  tle_busy = 0;           // 0 idle, 1 transfer running

void tle_recover_spi_dma(void);
