#pragma once

#define UART_BUFFER_SIZE    64

#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    16
#define UART2_RX_BUFFER_SIZE    64
#define UART2_TX_BUFFER_SIZE    64
#define UART3_RX_BUFFER_SIZE    64
#define UART3_TX_BUFFER_SIZE    64

typedef enum portMode_t {
    MODE_RX = 1,
    MODE_TX = 2,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_t;

typedef void (* serialReceiveCallbackPtr)(uint8_t data);   // used by serial drivers to return frames to app

typedef struct {
    portMode_t mode;
    uint32_t baudRate;
    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxDMAPos;
    bool txDMAEmpty;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;
    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;
    USART_TypeDef *USARTx;

    serialReceiveCallbackPtr callback;
} serialPort_t;

extern serialPort_t serialPort1;
extern serialPort_t serialPort2;
extern serialPort_t serialPort3;

serialPort_t *serialOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode);
uint8_t uartAvailable(serialPort_t *s);
uint8_t uartRead(serialPort_t *s);
void uartWrite(serialPort_t *s, uint8_t ch);
