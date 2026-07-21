/**
  ******************************************************************************
  * @file           : lora_bridge.c
  * @brief          : Bidirectional forwarding bridge implementation
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
static uint8_t usb_tx_temp[256];

static inline void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

static inline uint32_t RingBuffer_Available(const RingBuffer *rb) {
    return (rb->head - rb->tail) & (LORA_BRIDGE_BUF_SIZE - 1);
}

static inline uint32_t RingBuffer_FreeSpace(const RingBuffer *rb) {
    return LORA_BRIDGE_BUF_SIZE - 1 - RingBuffer_Available(rb);
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

    /* Ensure E28 M2 pin (PA0) is driven HIGH for transparent mode */
    HAL_GPIO_WritePin(LORA_M2_GPIO_Port, LORA_M2_Pin, GPIO_PIN_SET);

    /* Start non-blocking single-byte interrupt reception on USART2 */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

void LoraBridge_UsbRxCallback(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        RingBuffer_Put(&ring_usb_to_lora, buf[i]);
    }
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
        __IO uint32_t tmpreg;
        tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
        (void)tmpreg;
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void LoraBridge_Process(void) {
    /* 1. LoRa (USART2) -> USB CDC */
    uint32_t lora_avail = RingBuffer_Available(&ring_lora_to_usb);
    if (lora_avail > 0) {
        uint32_t chunk = (lora_avail > sizeof(usb_tx_temp)) ? sizeof(usb_tx_temp) : lora_avail;
        for (uint32_t i = 0; i < chunk; i++) {
            usb_tx_temp[i] = ring_lora_to_usb.buffer[(ring_lora_to_usb.tail + i) & (LORA_BRIDGE_BUF_SIZE - 1)];
        }

        uint8_t status = CDC_Transmit_FS(usb_tx_temp, (uint16_t)chunk);
        if (status == USBD_OK) {
            ring_lora_to_usb.tail = (ring_lora_to_usb.tail + chunk) & (LORA_BRIDGE_BUF_SIZE - 1);
        }
    }

    /* 2. USB CDC -> LoRa (USART2) */
    uint32_t usb_avail = RingBuffer_Available(&ring_usb_to_lora);
    if (usb_avail > 0) {
        GPIO_PinState aux_state = HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin);
        if (aux_state == GPIO_PIN_SET) {
            uint8_t uart_tx_temp[128];
            uint32_t chunk = (usb_avail > sizeof(uart_tx_temp)) ? sizeof(uart_tx_temp) : usb_avail;
            for (uint32_t i = 0; i < chunk; i++) {
                uart_tx_temp[i] = RingBuffer_Get(&ring_usb_to_lora);
            }
            HAL_UART_Transmit(&huart2, uart_tx_temp, (uint16_t)chunk, 500);
        }
    }
}
