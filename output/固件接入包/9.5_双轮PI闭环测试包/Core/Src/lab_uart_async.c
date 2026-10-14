#include "lab_uart_async.h"

#define TX_SIZE 512U
#define RX_SIZE 128U
#define FRAME_MAX 160U
static USART_TypeDef *port;
static volatile uint16_t tx_head, tx_tail, rx_head, rx_tail;
static uint8_t tx_buffer[TX_SIZE], rx_buffer[RX_SIZE];
static volatile uint8_t rx_fault;
static volatile uint32_t rx_errors, tx_dropped;

HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != USART1) return HAL_ERROR;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    port = uart->Instance;
    port->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE);
    tx_head = tx_tail = rx_head = rx_tail = 0;
    rx_fault = 0;
    rx_errors = tx_dropped = 0;
    /* SR followed by DR clears stale RXNE/ORE/FE/NE/PE on STM32F1. */
    volatile uint32_t status = port->SR;
    volatile uint32_t data = port->DR;
    (void)status; (void)data;
    port->CR3 |= USART_CR3_EIE;
    port->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U); /* TIM2 has priority 0. */
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __set_PRIMASK(mask);
    return HAL_OK;
}

int LabUART_TryWrite(const char *data, uint16_t length)
{
    if (port == NULL || data == NULL || length == 0U || length > FRAME_MAX) return 0;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint16_t free_bytes = (uint16_t)((tx_tail - tx_head - 1U) & (TX_SIZE - 1U));
    if (length > free_bytes) {
        ++tx_dropped; /* Drop a whole telemetry frame; never wait in control code. */
        __set_PRIMASK(mask);
        return 0;
    }
    uint16_t head = tx_head;
    for (uint16_t i = 0; i < length; ++i) {
        tx_buffer[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) & (TX_SIZE - 1U));
    }
    __DMB();
    tx_head = head;
    port->CR1 |= USART_CR1_TXEIE;
    __set_PRIMASK(mask);
    return 1;
}

int LabUART_TryRead(uint8_t *byte)
{
    if (byte == NULL || rx_tail == rx_head) return 0;
    *byte = rx_buffer[rx_tail];
    __DMB();
    rx_tail = (uint16_t)((rx_tail + 1U) & (RX_SIZE - 1U));
    return 1;
}

int LabUART_TakeRxFault(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    int fault = rx_fault != 0;
    if (fault) {
        rx_fault = 0;
        rx_tail = rx_head; /* Discard the incomplete command stream. */
    }
    __set_PRIMASK(mask);
    return fault;
}

uint32_t LabUART_RxErrors(void) { return rx_errors; }
uint32_t LabUART_TxDropped(void) { return tx_dropped; }

void LabUART_IRQHandler(void)
{
    if (port == NULL) return;
    uint32_t status = port->SR;
    uint32_t errors = status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE);
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)port->DR; /* SR/DR sequence clears error flags. */
        if (errors) {
            ++rx_errors;
            rx_fault = 1;
        } else if (status & USART_SR_RXNE) {
            uint16_t next = (uint16_t)((rx_head + 1U) & (RX_SIZE - 1U));
            if (next == rx_tail) {
                ++rx_errors;
                rx_fault = 1;
            } else {
                rx_buffer[rx_head] = byte;
                __DMB();
                rx_head = next;
            }
        }
    }
    if ((status & USART_SR_TXE) && (port->CR1 & USART_CR1_TXEIE)) {
        if (tx_tail != tx_head) {
            port->DR = tx_buffer[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1U) & (TX_SIZE - 1U));
        }
        if (tx_tail == tx_head) port->CR1 &= ~USART_CR1_TXEIE;
    }
}
