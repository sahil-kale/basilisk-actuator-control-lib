#ifndef HAL_SERIAL_HPP
#define HAL_SERIAL_HPP

#include <stdlib.h>

#include "hal_common.hpp"

namespace basilisk_hal {

// Create a new class for
class HAL_SERIAL {
   public:
    HAL_SERIAL() = default;
    virtual ~HAL_SERIAL() {}

    /**
     * @brief Transmit bytes over the serial port
     * @param byte Byte array buffer to transmit
     * @param size size of buffer
     * @return The status of the operation
     */
    virtual app_hal_status_E transmit(const void* byte, size_t size) = 0;

    /**
     * @brief Receive bytes over the serial port
     * @param byte Byte array buffer to receive
     * @param size size to receive. If the size is larger than the buffer, the return code will be
     * APP_HAL_BUFFER_DIMENSION_MISSMATCH. It will also update the size to the size of data read
     * @return The status of the operation
     */
    virtual app_hal_status_E receive(void* byte, size_t& size) = 0;

    // TODO: app_hal_status_E async_register_callback which can call comm_manager's process_received_data api
    virtual app_hal_status_E async_register_callback(void (*callback)(const uint8_t*, const size_t, void*),
                                                     const void* context) = 0;
};

}  // namespace basilisk_hal

#endif  // HAL_SERIAL