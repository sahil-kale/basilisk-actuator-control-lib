#ifndef COMM_MANAGER_HPP
#define COMM_MANAGER_HPP

#include <stdlib.h>

#include "comm_message_types.hpp"
#include "hal_serial.hpp"

namespace comm {
class CommManager {
   public:
    // TODO: if we want multiple serial ports, we can make this a vector of HAL_SERIAL
    CommManager(basilisk_hal::HAL_SERIAL& serial) : serial_(serial) {}

    /**
     * @brief Determine the metadata of a packet
     * @param buffer The buffer to read from
     * @param size The size of the buffer
     */
    static comm::generic_message_header_t determine_packet_metadata(uint8_t* buffer, size_t size);

    /**
     * @brief Create the metadata for a packet
     * @param channel_id The channel ID of the packet
     * @param message_length The length of the message
     * @param sequence The sequence number of the packet
     */
    static comm::generic_message_header_t create_packet_metadata(const comm::ChannelID channel_id, const uint16_t message_length,
                                                                 const uint8_t sequence);

    /**
     * @brief Create a buffer for a message header
     * @param header The header to create a buffer for
     * @param buffer The buffer to write to
     * @param buffer_size The size of the buffer
     */
    static bool create_message_header_buffer(const comm::generic_message_header_t header, uint8_t* buffer,
                                             const size_t buffer_size);

    /**
     * @brief Subscribe to a message channel and register a callback
     *
     * @returns Whether the subscription was successful
     */
    bool registerCallback(const comm::ChannelID channel_id, void (*callback)(comm::generic_message_header_t, const void*, void*),
                          void* context);

    /**
     * @brief Transmit a message on a given channel
     * @param channel_id The channel to transmit on
     * @param message The message to transmit
     * @returns Whether the transmission was successful
     */
    bool sendASAP(const comm::ChannelID channel_id, const comm::Message& message);

    /**
     * @brief Given a buffer of data, process it and dispatch it to the appropriate callback
     * @param buffer The buffer to process
     * @param size The size of the buffer
     * @note the buffer should be a complete datagram
     */
    void process_received_data(const uint8_t* buffer, const size_t size);

    /**f
     * @brief Given a buffer of data, process it and dispatch it to the appropriate callback
     * @param buffer The buffer to process
     * @param size The size of the buffer
     * @param context The context to pass to the callback
     */
    static void process_received_data_wrapper(const uint8_t* buffer, const size_t size, void* context) {
        CommManager* comm_manager = (CommManager*)context;
        if (comm_manager != nullptr) {
            comm_manager->process_received_data(buffer, size);
        }
    }

    // Constants
   private:
    static constexpr size_t kMaxSubscribers = 32;  // Maximum number of subscribers per channel ID
    typedef struct {
        // Function pointer to the callback
        void (*callback)(comm::generic_message_header_t, const void*, void*);
        // Context to pass to the callback
        void* context;
    } subscriber_t;

    // Create an array of subscribers for each channel ID up to the maximum number of subscribers
    subscriber_t subscribers_[static_cast<unsigned int>(comm::ChannelID::CHANNEL_COUNT)][CommManager::kMaxSubscribers] = {};

    size_t subscriber_count_[static_cast<unsigned int>(comm::ChannelID::CHANNEL_COUNT)] = {};
    basilisk_hal::HAL_SERIAL& serial_;

    // private static buffer for sendASAP messages
    uint8_t sendASAP_buffer_[MAX_MESSAGE_BUF_LEN];
};

}  // namespace comm

#endif  // COMM_MANAGER_HPP
