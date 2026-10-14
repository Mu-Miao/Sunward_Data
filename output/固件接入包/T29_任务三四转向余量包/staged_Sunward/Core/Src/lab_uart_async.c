#include "lab_uart_async.h"

#define TX_SIZE 512U
#define RX_SIZE 128U
#define FRAME_MAX 160U
typedef struct {
    USART_TypeDef *port;
    volatile uint16_t tx_head, tx_tail, rx_head, rx_tail;
    uint8_t tx_buffer[TX_SIZE], rx_buffer[RX_SIZE];
    volatile uint8_t rx_fault;
    volatile uint32_t rx_errors, tx_dropped;
} Channel;
static Channel channels[LAB_UART_COUNT];

static Channel *get_channel(LabUART_Port channel)
{
    return (unsigned)channel < LAB_UART_COUNT ? &channels[channel] : NULL;
}

HAL_StatusTypeDef LabUART_BeginPort(LabUART_Port channel, UART_HandleTypeDef *uart)
{
    Channel *c = get_channel(channel);
    if (c == NULL || uart == NULL) return HAL_ERROR;
    USART_TypeDef *expected = channel == LAB_UART_USB ? USART1 : USART3;
    if (uart->Instance != expected) return HAL_ERROR;
    IRQn_Type irq = channel == LAB_UART_USB ? USART1_IRQn : USART3_IRQn;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    c->port = uart->Instance;
    c->port->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE);
    c->tx_head = c->tx_tail = c->rx_head = c->rx_tail = 0;
    c->rx_fault = 0;
    c->rx_errors = c->tx_dropped = 0;
    /* SR followed by DR clears stale RXNE/ORE/FE/NE/PE on STM32F1. */
    volatile uint32_t status = c->port->SR;
    volatile uint32_t data = c->port->DR;
    (void)status; (void)data;
    c->port->CR3 |= USART_CR3_EIE;
    c->port->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(irq, 2U, 0U); /* TIM2 priority remains 0. */
    HAL_NVIC_ClearPendingIRQ(irq);
    HAL_NVIC_EnableIRQ(irq);
    __set_PRIMASK(mask);
    return HAL_OK;
}

int LabUART_TryWritePort(LabUART_Port channel, const char *data, uint16_t length)
{
    Channel *c = get_channel(channel);
    if (c == NULL || c->port == NULL || data == NULL || length == 0U || length > FRAME_MAX) return 0;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint16_t free_bytes = (uint16_t)((c->tx_tail - c->tx_head - 1U) & (TX_SIZE - 1U));
    if (length > free_bytes) {
        ++c->tx_dropped; /* Drop the whole frame; never wait for either UART. */
        __set_PRIMASK(mask);
        return 0;
    }
    uint16_t head = c->tx_head;
    for (uint16_t i = 0; i < length; ++i) {
        c->tx_buffer[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) & (TX_SIZE - 1U));
    }
    __DMB();
    c->tx_head = head;
    c->port->CR1 |= USART_CR1_TXEIE;
    __set_PRIMASK(mask);
    return 1;
}

int LabUART_TryReadPort(LabUART_Port channel, uint8_t *byte)
{
    Channel *c = get_channel(channel);
    if (c == NULL || byte == NULL || c->rx_tail == c->rx_head) return 0;
    *byte = c->rx_buffer[c->rx_tail];
    __DMB();
    c->rx_tail = (uint16_t)((c->rx_tail + 1U) & (RX_SIZE - 1U));
    return 1;
}

int LabUART_TakeRxFaultPort(LabUART_Port channel)
{
    Channel *c = get_channel(channel);
    if (c == NULL) return 0;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    int fault = c->rx_fault != 0;
    if (fault) {
        c->rx_fault = 0;
        c->rx_tail = c->rx_head;
    }
    __set_PRIMASK(mask);
    return fault;
}

uint32_t LabUART_RxErrorsPort(LabUART_Port channel)
{
    Channel *c = get_channel(channel);
    return c == NULL ? 0 : c->rx_errors;
}
uint32_t LabUART_TxDroppedPort(LabUART_Port channel)
{
    Channel *c = get_channel(channel);
    return c == NULL ? 0 : c->tx_dropped;
}

void LabUART_IRQHandlerPort(LabUART_Port channel)
{
    Channel *c = get_channel(channel);
    if (c == NULL || c->port == NULL) return;
    uint32_t status = c->port->SR;
    uint32_t errors = status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE);
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)c->port->DR;
        if (errors) {
            ++c->rx_errors;
            c->rx_fault = 1;
        } else if (status & USART_SR_RXNE) {
            uint16_t next = (uint16_t)((c->rx_head + 1U) & (RX_SIZE - 1U));
            if (next == c->rx_tail) {
                ++c->rx_errors;
                c->rx_fault = 1;
            } else {
                c->rx_buffer[c->rx_head] = byte;
                __DMB();
                c->rx_head = next;
            }
        }
    }
    if ((status & USART_SR_TXE) && (c->port->CR1 & USART_CR1_TXEIE)) {
        if (c->tx_tail != c->tx_head) {
            c->port->DR = c->tx_buffer[c->tx_tail];
            c->tx_tail = (uint16_t)((c->tx_tail + 1U) & (TX_SIZE - 1U));
        }
        if (c->tx_tail == c->tx_head) c->port->CR1 &= ~USART_CR1_TXEIE;
    }
}

HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef *u) { return LabUART_BeginPort(LAB_UART_USB, u); }
void LabUART_IRQHandler(void) { LabUART_IRQHandlerPort(LAB_UART_USB); }
int LabUART_TryWrite(const char *d, uint16_t n) { return LabUART_TryWritePort(LAB_UART_USB, d, n); }
int LabUART_TryRead(uint8_t *b) { return LabUART_TryReadPort(LAB_UART_USB, b); }
int LabUART_TakeRxFault(void) { return LabUART_TakeRxFaultPort(LAB_UART_USB); }
uint32_t LabUART_RxErrors(void) { return LabUART_RxErrorsPort(LAB_UART_USB); }
uint32_t LabUART_TxDropped(void) { return LabUART_TxDroppedPort(LAB_UART_USB); }
