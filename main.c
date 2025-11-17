#include "uart.h"
#include <stdio.h>



int main(void) {
    Uart_init();
    DMA_init();
    
    uint8_t data[] = "DMA test";
    DMA_send(data, sizeof(data) - 1);
    
    while(1) {
        

    }
    
    return 0;
}