#include "tle5012_dma.h"



uint16_t reg_word;

//volatile tle_state_t tle_state;
volatile tle_pkt_t pkt;
volatile tle_state_t tle_state =TLE_IDLE;

//volatile uint16_t tle_angle_latest = 0;   // 0..65535 mapped


static inline void CS_L(void){ HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); }

bool tle_read_start_dma(void)
{
    if (tle_state != TLE_IDLE) return false;

    const uint16_t len = sizeof(pkt) / sizeof(uint16_t);

    reg_word = (UINT16_C(1)    << 15) |   // RW=Read
               (UINT16_C(0x0)  << 11) |   // Lock
               (UINT16_C(0x0)  << 10) |   // UPD=Buffer
               (UINT16_C(0x02) << 4)  |   // ADDR
               (len - 1);                // ND

    tle_state = TLE_TX;
    SPI_1LINE_TX(&hspi3);
    CS_L();

    // 16-bit SPI => Size=1 sends one 16-bit word
    if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)&reg_word, 1) != HAL_OK) {

        tle_state = TLE_ERR;
        CS_H();
        return false;
    }
    return true;
}

void tle_recover_spi_dma(void)
{
    // Always release CS
    CS_H();

    // Abort any ongoing SPI/DMA transfers (HAL will disable DMA requests too)
    (void)HAL_SPI_Abort(&hspi3);

    // If Abort isn't enough (rare), force-disable DMA streams
    if (hspi3.hdmarx) __HAL_DMA_DISABLE(hspi3.hdmarx);
    if (hspi3.hdmatx) __HAL_DMA_DISABLE(hspi3.hdmatx);

    // Clear SPI overrun if set (read DR then SR)
    if (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_OVR)) {
        (void)hspi3.Instance->DR;
        (void)hspi3.Instance->SR;
    }

    // Wait for not busy
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY)) {}

    // Optional hard reset of SPI peripheral (very effective)
    __HAL_SPI_DISABLE(&hspi3);
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();
    __HAL_SPI_ENABLE(&hspi3);

    // Reset your driver state
    tle_state = TLE_IDLE;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi3) return;
    if (tle_state != TLE_TX) return;

    tle_state = TLE_RX;

    const uint16_t len = sizeof(pkt) / sizeof(uint16_t);
   SPI_1LINE_RX(&hspi3);

    // 16-bit SPI => Size=len receives len 16-bit words
    if (HAL_SPI_Receive_DMA(&hspi3, (uint8_t*)&pkt, len) != HAL_OK) {
        tle_state = TLE_ERR;
        CS_H();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi3) return;
    CS_H();

    // Ensure transaction fully ended
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY)) {}

    if (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_OVR)) {
        (void)hspi3.Instance->DR;
        (void)hspi3.Instance->SR;
    }

    tle_state = TLE_DONE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi3) return;
    CS_H();
    tle_state = TLE_ERR;
}


