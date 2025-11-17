#include "uart.h"
#include <stdio.h>

sUart_Frame_transmit gTx_Buffer;
uint8_t uart_header[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t rx_buffer[256];

int Led_init(void){
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; // Enable GPIOB clock

    GPIOB->MODER &= ~(0x3 << (0 * 2)); // Clear mode for PB0
    GPIOB->MODER |= (0x1 << (0 * 2)); // Set PB0 to output

    return 0;
};

int Uart_init(void){
    RCC->AHB4ENR|= 0x08; //Enable clock for GPIO port D
    RCC->APB1LENR|= 0x40000; //Enable clock for USART3

    GPIOD->MODER &= 0xFFF0FFFF; //Clear PD8 PD9
    GPIOD->MODER |= 0x000A0000; //Set alternate function mode for PD8-PD9
    GPIOD->PUPDR &= 0xFFFCFFFF; //clear
    GPIOD->PUPDR |= 0x00010000; //Enable pull-up for PD8
    GPIOD_AFRH &= 0xFFFFFF00; //clear
    GPIOD_AFRH |= 0x00000077; //PA9-PA10 set to AF7 for USART1

    USART3->BRR= 0x1A0A; //baud rate: 16000000/1666.6667=9600
    USART3->CR1|= 0x8; //transmitter enable
    USART3->CR1|= 0x4; // receiver enable
    USART3->CR1|= 0x1; // USART enable

    return 0;
};

int DMA_init(void){
    RCC->AHB1ENR|=0x01; //DMA 1 clock enable

    DMAMUX1_Channel1->CCR&=0xFFFFFF80; //Clear
    DMAMUX1_Channel1->CCR|=0x0000002E; //request 46 for tx

    DMA1_Stream1->CR=0; //peripheral data size:8bit, memory data size: 8bit,priority level low
    DMA1_Stream1->CR|=0x40; //Memory to peripheral mode
    DMA1_Stream1->CR|=0x400; //Memory increment mode
    //DMA1_Stream1->CR|=0x10; //Transfer complete interrupt enable
    DMA1_Stream1->CR|=0x00100000; //Enable bufferable transfer
    DMA1_Stream1->FCR = 0x00; //Direct mode no FIFO
    //assigning TX address to peripheral address
    DMA1_Stream1->PAR = (unsigned long)&USART3->TDR;

    USART3->CR3|=0x80; //Enable DMA transmit



	DMAMUX1_Channel0->CCR&=0xFFFFFF80;	//Clear
	DMAMUX1_Channel0->CCR|=0x0000002D;	//request 45 for rx
	DMA1_Stream0->CR=0;
	DMA1_Stream0->CR|=0x00;				//Memory to peripheral mode
	DMA1_Stream0->CR|=0x400;			//Memory increment mode
	//DMA1_Stream0->CR|=0x10;			//Transfer complete interrupt enable
	DMA1_Stream0->CR|=0x00100000;		//Enable bufferable transfer
	DMA1_Stream0->FCR = 0x00; 			//Direct mode no FIFO
	DMA1_Stream0->PAR = (unsigned long)&USART3->RDR;//assigning RX address to peripheral address
	DMA1_Stream0->M0AR=(unsigned long)&rx_buffer;
	DMA1_Stream0->NDTR=sizeof(rx_buffer);
	USART3->CR3|=0x40;					//Enable DMA receive

	USART3->CR1|= 0x10;					//Enable idle interrupt
	DMA1_Stream0->CR|=0x1; //enable stream
	NVIC_EnableIRQ(USART3_IRQn);
    return 0;
};

int DMA_send(unsigned char* data, unsigned int length){
    memset(&gTx_Buffer, 0, sizeof(gTx_Buffer));

    // Copy header
    memcpy(gTx_Buffer.header, uart_header, 8);

    // Set payload length
    gTx_Buffer.payload_length = (__uint16_t)length;

    // Copy payload
    memcpy(gTx_Buffer.payload, data, length);

    while((DMA1_Stream1->CR & 0x1)==0x1);
    DMA1->LIFCR|=0x800; //Clear transfer complete interrupt flag,stream 1
    DMA1_Stream1->M0AR=(unsigned long)&gTx_Buffer;
    DMA1_Stream1->NDTR=length+10;
    DMA1_Stream1->CR|=0x1; //enable stream
    return 0;
};



void USART3_IRQHandler(void){
    if(USART3->ISR & 0x10){                    // IDLE flag
        USART3->ICR |= 0x10;                   // Clear IDLE flag

        DMA1_Stream0->CR &= ~0x1;              // Disable DMA

        uint16_t received_length = sizeof(rx_buffer) - DMA1_Stream0->NDTR;

        if(received_length > 0){

        	printf("RLength: %d\n",received_length);
        	fflush(stdout);
        }

        // Restart DMA for next reception
        DMA1_Stream0->NDTR = sizeof(rx_buffer);
        DMA1_Stream0->CR |= 0x1;               // Enable stream
    }
};