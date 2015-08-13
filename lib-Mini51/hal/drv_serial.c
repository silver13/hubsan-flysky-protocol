#include "hal.h"
#include "drv_serial.h"

//#define USE_PERIPH_BUFFERS

serialPort_t serialPort1;

// UART1 - Configuration
serialPort_t *serialUART1(uint32_t baudRate, portMode_t mode)
{
    serialPort_t *s;
#if !defined(USE_PERIPH_BUFFERS)
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
#endif
    s = &serialPort1;
#if !defined(USE_PERIPH_BUFFERS)
    s->rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->txBufferSize = UART1_TX_BUFFER_SIZE;
    s->rxBuffer = rx1Buffer;
    s->txBuffer = tx1Buffer;
#endif
    s->UARTx = UART; // extraneous

    // UART RXD P12 and TXD P13
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);

    return s;
}

serialPort_t *serialOpen(UART_T *UARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode)
{
    serialPort_t *s = NULL;

    if (UARTx == UART)
        s = serialUART1(baudRate, mode);

    s->UARTx = UARTx;
    
    // Clear the FIFOs
    UARTx->FSR |= (UART_FCR_TFR_Msk | UART_FCR_RFR_Msk);
    s->rxBufferHead = s->rxBufferTail = 0;
    s->txBufferHead = s->txBufferTail = 0;
    // callback for IRQ-based RX ONLY
    s->callback = callback;
    s->mode = mode;
    s->baudRate = baudRate;

    // For simplicity, ignore word length, stop bits, parity, mode
    // If we really need it, use UART_SetLine_Config, and for mode,
    // use MFP to disable either TX or RX
    UART_Open(UARTx, baudRate);
    // If we use UART_SetLine_Config, we need this:
    // UARTx->FUN_SEL = UART_FUNC_SEL_UART; 


    if ((mode & MODE_RX)
#if defined(USE_PERIPH_BUFFERS)
        && callback
#endif
        ) {
        // Rx ready interrupt and buffer error interrupt
        // Buffer error handles hardware RX buffer overflow,
        // otherwise RX stops receiving
        UART_EnableInt(UARTx, UART_IER_RDA_IEN_Msk | UART_IER_BUF_ERR_IEN_Msk);
    }

    return s;
}

uint8_t uartAvailable(serialPort_t *s)
{
//    return (s->UARTx->FSR & UART_FSR_RX_POINTER_Msk) >> UART_FSR_RX_POINTER_Pos;
#if defined(USE_PERIPH_BUFFERS)
    return !(UART->FSR & UART_FSR_RX_EMPTY_Msk);
#else
    return (s->rxBufferHead - s->rxBufferTail) % s->rxBufferSize;
#endif
}

uint8_t uartRead(serialPort_t *s)
{
#if defined(USE_PERIPH_BUFFERS)
    return UART_READ(s->UARTx);
#else
    uint8_t ch;

    ch = s->rxBuffer[s->rxBufferTail];
    s->rxBufferTail = (s->rxBufferTail + 1) % s->rxBufferSize;

    return ch;
#endif
}

void uartWrite(serialPort_t *s, uint8_t ch)
{
#if defined(USE_PERIPH_BUFFERS)
    UART_WRITE(s->UARTx, ch);
#else
    // Wait here if buffer is full
    uint32_t nextHead = (s->txBufferHead + 1) % s->txBufferSize;
    while (nextHead == s->txBufferTail) ;
    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = nextHead;

    // Enable transmit by enabling TX empty interrupt
    UART_EnableInt(s->UARTx, UART_IER_THRE_IEN_Msk);
#endif
}


void uartInit()
{
    CLK_EnableModuleClock(UART_MODULE);

    // UART clock source
//    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_IRC22M,CLK_CLKDIV_UART(1));
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_IRC22M;// Clock source from internal 22.1184MHz RC clock
    CLK->CLKDIV  &= ~CLK_CLKDIV_UART_N_Msk;
    CLK->CLKDIV  |= CLK_CLKDIV_UART(1);
//    SYS_ResetModule(SYS_IPRSTC2_UART_RST_Msk);
}



// Handlers

// USART2 Rx/Tx IRQ Handler
void UART_IRQHandler(void)
{
    serialPort_t *s = &serialPort1;
    UART_T *uart = s->UARTx;
    uint16_t isr = uart->ISR;

    if (isr & UART_ISR_RDA_INT_Msk) {
        /* Get all the input characters */
        while(uart->ISR & UART_ISR_RDA_IF_Msk) {
            /* Get the character from UART Buffer */
            uint8_t u8InChar = uart->RBR;
            // If we registered a callback, pass crap there
            if (s->callback) {
                s->callback(u8InChar);
            } else {
                s->rxBuffer[s->rxBufferHead] = u8InChar;
                s->rxBufferHead = (s->rxBufferHead + 1) % s->rxBufferSize;
            }
        }
    }
    if (isr & UART_ISR_THRE_INT_Msk) {
        if (s->txBufferTail != s->txBufferHead) {
            UART_WRITE(uart, s->txBuffer[s->txBufferTail]);
            s->txBufferTail = (s->txBufferTail + 1) % s->txBufferSize;
        } else {
            // Don't use UART_DisableInt here, it will disable
            // ALL interrupts from UART
            uart->IER &= ~UART_IER_THRE_IEN_Msk;
        }
    }
    if (isr & UART_ISR_BUF_ERR_INT_Msk) {
        if (uart->FSR | UART_FSR_RX_OVER_IF_Msk) {
            // Clear receive buffer - can't rely on it's content
            // after overflow
//            s->rxBufferHead = s->rxBufferTail = 0;
        }
        uart->FSR |= UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk;
    }
}
