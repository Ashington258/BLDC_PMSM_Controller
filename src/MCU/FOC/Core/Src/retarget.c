// retarget.c
#include "usart.h" // 需要 CubeMX 生成的 huart1
#include <stdint.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

/* GNU newlib 的最低层写接口：
   printf/puts 等最终会走到 _write() */
int _write(int file, char *ptr, int len)
{
    (void)file;
    // 阻塞发送——简单稳妥，调试足够。如果你在高频 ISR 里打日志会卡，见文末“DMA 方案”
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* 可选：把 \n 自动补 \r（看你终端需求）
#include <string.h>
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; ++i) {
        uint8_t ch = (uint8_t)ptr[i];
        if (ch == '\n') {
            uint8_t crlf[2] = {'\r','\n'};
            HAL_UART_Transmit(&huart1, crlf, 2, HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
        }
    }
    return len;
}
*/
