#ifndef LOG_H
#define LOG_H

#include "SEGGER_RTT.h"

// 日志级别定义
#define LOG_LEVEL_NOLOG 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4

// 当前日志级别配置
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

// 可选：定义默认通道
#ifndef LOG_CHANNEL
#define LOG_CHANNEL 0
#endif

// 可选：是否启用时间戳（1=启用，0=禁用）
#ifndef LOG_ENABLE_TIMESTAMP
#define LOG_ENABLE_TIMESTAMP 0
#endif

// 获取时间戳接口：
// 默认使用 HAL_GetTick()，如果无 HAL 库请在编译选项或工程中定义 LOG_GET_TIMESTAMP() 为合适函数
#ifndef LOG_GET_TIMESTAMP
// #include "stm32f7xx_hal.h" // 用户可移植，根据实际平台调整或自行实现
#define LOG_GET_TIMESTAMP() HAL_GetTick()
#endif

// 内部时间戳宏：格式 seconds.milliseconds
#if LOG_ENABLE_TIMESTAMP
#define _LOG_TIMESTAMP()                                         \
    do                                                           \
    {                                                            \
        uint32_t _ts = LOG_GET_TIMESTAMP();                      \
        uint32_t _s = _ts / 1000u;                               \
        uint32_t _ms = _ts % 1000u;                              \
        SEGGER_RTT_printf(LOG_CHANNEL, "[%lu.%03lu] ", _s, _ms); \
    } while (0)
#else
#define _LOG_TIMESTAMP() \
    do                   \
    {                    \
    } while (0)
#endif

// 彩色控制宏
#define LOG_RESET RTT_CTRL_RESET
#define LOG_RED RTT_CTRL_TEXT_RED
#define LOG_GREEN RTT_CTRL_TEXT_GREEN
#define LOG_YELLOW RTT_CTRL_TEXT_YELLOW
#define LOG_BLUE RTT_CTRL_TEXT_BLUE
#define LOG_MAGENTA RTT_CTRL_TEXT_MAGENTA
#define LOG_CYAN RTT_CTRL_TEXT_CYAN
#define LOG_WHITE RTT_CTRL_TEXT_WHITE

// 基础输出封装
#define LOG_OUT(channel, fmt, ...) SEGGER_RTT_printf(channel, fmt, ##__VA_ARGS__)

// 调试宏实现
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOG(fmt, ...)                                                          \
    do                                                                         \
    {                                                                          \
        _LOG_TIMESTAMP();                                                      \
        LOG_OUT(LOG_CHANNEL, LOG_RESET "[DEBUG]: " fmt "\r\n", ##__VA_ARGS__); \
    } while (0)
#else
#define LOG(fmt, ...) \
    do                \
    {                 \
    } while (0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(fmt, ...)                                                    \
    do                                                                        \
    {                                                                         \
        /* 不打印时间戳，仅输出信息 */                                        \
        LOG_OUT(LOG_CHANNEL, LOG_GREEN "[信息]: " fmt "\r\n", ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_INFO(fmt, ...) \
    do                     \
    {                      \
    } while (0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(fmt, ...)                                                     \
    do                                                                         \
    {                                                                          \
        _LOG_TIMESTAMP();                                                      \
        LOG_OUT(LOG_CHANNEL, LOG_YELLOW "[警告]: " fmt "\r\n", ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_WARN(fmt, ...) \
    do                     \
    {                      \
    } while (0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(fmt, ...)                                                 \
    do                                                                      \
    {                                                                       \
        _LOG_TIMESTAMP();                                                   \
        LOG_OUT(LOG_CHANNEL, LOG_RED "[错误]: " fmt "\r\n", ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_ERROR(fmt, ...) \
    do                      \
    {                       \
    } while (0)
#endif

// 可选：无格式化装饰的原始输出
#define LOG_RAW(fmt, ...)                         \
    do                                            \
    {                                             \
        LOG_OUT(LOG_CHANNEL, fmt, ##__VA_ARGS__); \
    } while (0)

#endif // LOG_H
