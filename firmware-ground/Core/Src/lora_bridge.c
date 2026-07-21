/**
  ******************************************************************************
  * @file           : lora_bridge.c
  * @brief          : Robust bidirectional forwarding bridge implementation
  ******************************************************************************
  */

#include "lora_bridge.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Ring buffer structure */
typedef struct {
    uint8_t buffer[LORA_BRIDGE_BUF_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer;

static RingBuffer ring_lora_to_usb;
static RingBuffer ring_usb_to_lora;

static uint8_t uart_rx_byte;
static uint8_t usb_tx_buffer[256];
static volatile uint8_t usb_tx_busy = 0;

static inline void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

static inline uint32_t RingBuffer_Available(const RingBuffer *rb) {
    return (rb->head - rb->tail) & (LORA_BRIDGE_BUF_SIZE - 1);
}

static inline void RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    uint32_t next = (rb->head + 1) & (LORA_BRIDGE_BUF_SIZE - 1);
    if (next != rb->tail) {
        rb->buffer[rb->head] = data;
        rb->head = next;
    }
}

static inline uint8_t RingBuffer_Get(RingBuffer *rb) {
    uint8_t data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & (LORA_BRIDGE_BUF_SIZE - 1);
    return data;
}

void LoraBridge_Init(void) {
    RingBuffer_Init(&ring_lora_to_usb);
    RingBuffer_Init(&ring_usb_to_lora);
    usb_tx_busy = 0;

    /* Drive E28 M2 pin (PA0) HIGH for transparent mode */
    HAL_GPIO_WritePin(LORA_M2_GPIO_Port, LORA_M2_Pin, GPIO_PIN_SET);

    /* Clear any initial UART error flags */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    huart2.ErrorCode = HAL_UART_ERROR_NONE;
    huart2.RxState = HAL_UART_STATE_READY;

    /* Start non-blocking single-byte interrupt reception on USART2 */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

void LoraBridge_UsbRxCallback(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        RingBuffer_Put(&ring_usb_to_lora, buf[i]);
    }
}

void LoraBridge_UsbTxCpltCallback(void) {
    usb_tx_busy = 0;
}

void LoraBridge_UartRxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        RingBuffer_Put(&ring_lora_to_usb, uart_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    LoraBridge_UartRxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        /* Clear all hardware error flags */
        __IO uint32_t tmpreg;
        tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
        (void)tmpreg;

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;

        /* Re-arm interrupt reception */
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void LoraBridge_Process(void) {
    /* -------------------------------------------------------------
     * 1. LoRa (USART2 RX) -> USB CDC (PC)
     * ------------------------------------------------------------- */
    if (!usb_tx_busy) {
        __disable_irq();
        uint32_t lora_avail = RingBuffer_Available(&ring_lora_to_usb);
        __enable_irq();

        if (lora_avail > 0) {
            uint32_t chunk = (lora_avail > sizeof(usb_tx_buffer)) ? sizeof(usb_tx_buffer) : lora_avail;

            for (uint32_t i = 0; i < chunk; i++) {
                usb_tx_buffer[i] = RingBuffer_Get(&ring_lora_to_usb);
            }

            usb_tx_busy = 1;
            uint8_t status = CDC_Transmit_FS(usb_tx_buffer, (uint16_t)chunk);
            if (status != USBD_OK) {
                usb_tx_busy = 0; /* If USB transmit failed/busy, release flag */
            }
        }
    }

    /* -------------------------------------------------------------
     * 2. USB CDC (PC) -> LoRa (USART2 TX)
     * ------------------------------------------------------------- */
    uint32_t usb_avail = RingBuffer_Available(&ring_usb_to_lora);
    if (usb_avail > 0) {
        /* Check AUX pin (PA1): High = Idle. If Low, wait up to 100ms timeout fallback */
        GPIO_PinState aux_state = HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin);
        static uint32_t aux_low_start = 0;

        if (aux_state == GPIO_PIN_RESET) {
            if (aux_low_start == 0) {
                aux_low_start = HAL_GetTick();
            }
        } else {
            aux_low_start = 0;
        }

        /* Transmit if AUX is High OR if AUX has been Low for > 100ms (timeout fallback) */
        if (aux_state == GPIO_PIN_SET || (aux_low_start != 0 && (HAL_GetTick() - aux_low_start > 100))) {
            uint8_t uart_tx_temp[128];
            uint32_t chunk = (usb_avail > sizeof(uart_tx_temp)) ? sizeof(uart_tx_temp) : usb_avail;
            for (uint32_t i = 0; i < chunk; i++) {
                uart_tx_temp[i] = RingBuffer_Get(&ring_usb_to_lora);
            }
            HAL_UART_Transmit(&huart2, uart_tx_temp, (uint16_t)chunk, 500);
            aux_low_start = 0;
        }
    }
}
