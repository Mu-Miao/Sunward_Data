/* Paste this function into main.c: USER CODE BEGIN 0.
 * Requires the existing main.h, usart.h and stdio.h includes.
 * Call BT_TestRun() immediately after Lab_MotorStartBrake().
 * Standalone diagnostic: never returns; send ASCII lines ending in LF.
 */
static void BT_TestRun(void)
{
    uint8_t ch;
    char line[64];
    char reply[96];
    uint16_t used = 0U;
    uint8_t discard = 0U;
    uint8_t boot[] = "\r\nBT TEST: USART3 9600; send hello + LF\r\n";

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    HAL_UART_Transmit(&huart1, boot, sizeof boot - 1U, 100U);

    while (1) {
        HAL_StatusTypeDef status =
            HAL_UART_Receive(&huart3, &ch, 1U, 20U);

        if (status != HAL_OK) {
            if (status == HAL_ERROR) {
                __HAL_UART_CLEAR_OREFLAG(&huart3);
                used = 0U;
                discard = 1U;
            }
            continue;
        }

        if (ch == '\r') continue;

        if (ch == '\n') {
            int n = 0;
            if (discard) {
                n = snprintf(reply, sizeof reply,
                             "ERR: line too long or RX error\r\n");
            } else if (used > 0U) {
                line[used] = '\0';
                n = snprintf(reply, sizeof reply, "ECHO: %s\r\n", line);
            }
            if (n > 0 && n < (int)sizeof reply) {
                HAL_UART_Transmit(&huart3, (uint8_t *)reply,
                                  (uint16_t)n, 200U);
                HAL_UART_Transmit(&huart1, (uint8_t *)reply,
                                  (uint16_t)n, 100U);
            }
            used = 0U;
            discard = 0U;
        } else if (!discard) {
            if (used < sizeof line - 1U) {
                line[used++] = (char)ch;
            } else {
                discard = 1U;
            }
        }
    }
}
