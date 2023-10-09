#ifndef COMM_message_channelS_HPP
#define COMM_message_channelS_HPP
#include <stdint.h>

static const size_t MAX_MESSAGE_BUF_LEN = 256;

namespace comm {

using channel_id_t = size_t;

// Define an enum class for message channel ID's
enum class ChannelID : channel_id_t {
    ARM_DISARM = 0,
    SET_SPEED = 1,
    SYSTEM_MANAGER_STATE = 2,
    CHANNEL_COUNT = 3,
};

// Define a virtual class for a message
class Message {
   public:
    Message(uint16_t msg_len) : len(msg_len){};
    ~Message() = default;
    /**
     * @brief Set the buffer of the message and translate it into the message
     * @param buffer The buffer to read from
     * @param len The length of the buffer
     * @returns Whether the buffer was successfully set
     */
    virtual bool populateDataFromBuffer(const uint8_t* buffer, const uint16_t len) = 0;

    /**
     * @brief get the buffer of the message for use when sending the message
     * @returns The buffer of the message
     */
    virtual const void* getBuffer() const = 0;
    uint16_t len;
};

typedef struct __attribute__((__packed__)) generic_message_header {
    uint8_t SOF;
    uint16_t message_channel;
    uint16_t message_length;
    uint8_t message_sequence;
} generic_message_header_t;

// Define an arm/disarm message
class ArmDisarmMessage : public Message {
   public:
    typedef struct __attribute__((__packed__)) arm_disarm_message {
        bool arm;
    } arm_disarm_message_t;

    arm_disarm_message_t data;
    ArmDisarmMessage() : Message(sizeof(arm_disarm_message_t)) { data.arm = false; }
    ArmDisarmMessage(bool arm) : Message(sizeof(arm_disarm_message_t)) { data.arm = arm; }
    const void* getBuffer() const override { return (void*)&data; }

    bool populateDataFromBuffer(const uint8_t* buffer, const uint16_t len) override {
        bool ret = false;
        if (len == sizeof(arm_disarm_message_t)) {
            data = *(arm_disarm_message_t*)buffer;
            ret = true;
        }

        return ret;
    }
};

// Define a set speed message
class SetSpeedMessage : public Message {
   public:
    typedef struct __attribute__((__packed__)) set_speed_message {
        float speed;
    } set_speed_message_t;

    set_speed_message_t data;
    SetSpeedMessage() : Message(sizeof(set_speed_message_t)) { data.speed = 0.0f; }
    SetSpeedMessage(float speed) : Message(sizeof(set_speed_message_t)) { data.speed = speed; }
    const void* getBuffer() const override { return (void*)&data; }
    bool populateDataFromBuffer(const uint8_t* buffer, const uint16_t len) override {
        bool ret = false;
        if (len == sizeof(set_speed_message_t)) {
            data = *(set_speed_message_t*)buffer;
            ret = true;
        }

        return ret;
    }
};

// Define a message that describes the state of the SystemManager
class SystemStateMessage : public Message {
   public:
    typedef struct __attribute__((__packed__)) system_state_message {
        bool armed;
        float speed;
    } system_state_message_t;

    system_state_message_t data;
    SystemStateMessage() : Message(sizeof(system_state_message_t)) {
        data.armed = false;
        data.speed = 0.0f;
    }
    SystemStateMessage(bool armed, float speed) : Message(sizeof(system_state_message_t)) {
        data.armed = armed;
        data.speed = speed;
    }
    const void* getBuffer() const override { return (void*)&data; }
    bool populateDataFromBuffer(const uint8_t* buffer, const uint16_t len) override {
        bool ret = false;
        if (len == sizeof(system_state_message_t)) {
            data = *(system_state_message_t*)buffer;
            ret = true;
        }

        return ret;
    }
};

}  // namespace comm

#endif  // COMM_message_channelS_HPP