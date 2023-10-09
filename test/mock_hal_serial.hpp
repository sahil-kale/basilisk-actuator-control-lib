#ifndef MOCK_HAL_SERIAL_HPP
#define MOCK_HAL_SERIAL_HPP

#include "gmock/gmock.h"
#include "hal_serial.hpp"

namespace basilisk_hal {

class MockSerial : public HAL_SERIAL {
   public:
    MockSerial() = default;
    ~MockSerial() override = default;

    MOCK_METHOD(app_hal_status_E, transmit, (const void* byte, size_t size), (override));
    MOCK_METHOD(app_hal_status_E, receive, (void* byte, size_t& size), (override));
    MOCK_METHOD(app_hal_status_E, async_register_callback,
                (void (*callback)(const uint8_t*, const size_t, void*), const void* context), (override));
};

}  // namespace basilisk_hal

#endif  // MOCK_HAL_SERIAL_HPP