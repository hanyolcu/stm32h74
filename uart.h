#ifndef UART_H
#define UART_H

#include "stm32h7xx.h"
#include <stdint.h>
#include <string.h>

// Helper macros
#define GPIOD_AFRH (GPIOD->AFR[1])

// Frame structure for UART transmission
typedef struct {
    uint8_t header[8];
    uint16_t payload_length;
    uint8_t payload[246];  // 256 - 8 (header) - 2 (length)
} sUart_Frame_transmit;

// External variables
extern sUart_Frame_transmit gTx_Buffer;
extern uint8_t uart_header[8];
extern uint8_t rx_buffer[256];

// Function prototypes
int Led_init(void);
int Uart_init(void);
int DMA_init(void);
int DMA_send(unsigned char* data, unsigned int length);
void USART3_IRQHandler(void);

#endif // UART_H