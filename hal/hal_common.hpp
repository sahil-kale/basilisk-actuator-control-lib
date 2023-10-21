#ifndef HAL_COMMON_HPP
#define HAL_COMMON_HPP

#include <stdint.h>

typedef enum {
    APP_HAL_OK,
    APP_HAL_NOT_INITIALIZED,
    APP_HAL_BUSY,
    APP_HAL_ERROR,
    APP_HAL_TIMEOUT,
    APP_HAL_NOT_IMPLEMENTED,
    APP_HAL_BUFFER_DIMENSION_MISSMATCH,
} app_hal_status_E;

typedef int64_t utime_t;

#endif