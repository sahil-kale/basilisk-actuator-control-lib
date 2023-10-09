#include "comm_manager.hpp"

#include <string.h>

#include "hal_common.hpp"

namespace comm {

generic_message_header_t CommManager::determine_packet_metadata(uint8_t* buffer, size_t size) {
    generic_message_header_t header = {0x00, 0x0000, 0x0000, 0x00};
    if (size >= 4) {
        header.SOF = buffer[0];
        header.message_channel = (buffer[1] << 8) | buffer[2];
        header.message_length = (buffer[3] << 8) | buffer[4];
        header.message_sequence = buffer[5];
    }
    return header;
}

generic_message_header_t CommManager::create_packet_metadata(const comm::ChannelID channel_id, const uint16_t message_length,
                                                             const uint8_t sequence) {
    generic_message_header_t header;
    header.SOF = 0x00;
    header.message_channel = static_cast<uint16_t>(channel_id);
    header.message_length = message_length;
    header.message_sequence = sequence;

    return header;
}

bool CommManager::create_message_header_buffer(const comm::generic_message_header_t header, uint8_t* buffer,
                                               const size_t buffer_size) {
    bool ret = false;
    if (buffer_size >= sizeof(generic_message_header_t)) {
        buffer[0] = header.SOF;
        buffer[1] = header.message_channel >> 8;
        buffer[2] = header.message_channel & 0xFF;
        buffer[3] = header.message_length >> 8;
        buffer[4] = header.message_length & 0xFF;
        buffer[5] = header.message_sequence;
        ret = true;
    }
    return ret;
}

bool CommManager::registerCallback(const comm::ChannelID channel_id,
                                   void (*callback)(comm::generic_message_header_t, const void*, void*), void* context) {
    bool ret = false;
    // get the number of subscribers for the channel
    size_t num_subscribers = subscriber_count_[static_cast<unsigned int>(channel_id)];
    // check if the number of subscribers is less than the max
    if (num_subscribers < kMaxSubscribers) {
        // add the callback to the subscribers array
        subscriber_t* subscriber_ = &subscribers_[static_cast<unsigned int>(channel_id)][num_subscribers];
        subscriber_->callback = callback;
        subscriber_->context = context;
        // increment the number of subscribers
        subscriber_count_[static_cast<unsigned int>(channel_id)]++;
        ret = true;
    }

    return ret;
}

void CommManager::process_received_data(const uint8_t* buffer, const size_t size) {
    size_t i = 0;
    while (i < size) {
        // get the header
        generic_message_header_t header = determine_packet_metadata((uint8_t*)buffer + i, size - i);
        // get the channel id
        channel_id_t channel_id = static_cast<channel_id_t>(header.message_channel);
        // get the message length
        uint16_t message_length = header.message_length;
        // get the sequence
        uint8_t sequence = header.message_sequence;
        (void)(sequence);  // UNUSED

        if (message_length > size - i) {
            // the message length is too long, so break out of the loop
            break;
        } else if (channel_id >= static_cast<unsigned int>(ChannelID::CHANNEL_COUNT)) {
            // the channel id is invalid, so break out of the loop
            break;
        }

        // Fire the callback for the channel
        for (size_t j = 0; j < subscriber_count_[static_cast<unsigned int>(channel_id)]; j++) {
            subscriber_t* subscriber_ = &subscribers_[static_cast<unsigned int>(channel_id)][j];
            subscriber_->callback(header, buffer + i + sizeof(generic_message_header_t), subscriber_->context);
        }

        // increment i by the message length and the size of the header
        i += message_length + sizeof(generic_message_header_t);
    }
}

bool CommManager::sendASAP(const comm::ChannelID channel_id, const comm::Message& message) {
    bool ret = false;
    // get the buffer
    const void* buffer = message.getBuffer();
    // get the length
    uint16_t len = message.len;

    // create the header
    generic_message_header_t header = CommManager::create_packet_metadata(channel_id, len, 0U);

    // copy the header into the buffer
    memcpy(sendASAP_buffer_, &header, sizeof(generic_message_header_t));
    // copy the message into the buffer
    memcpy(sendASAP_buffer_ + sizeof(generic_message_header_t), buffer, len);

    // transmit the buffer
    if (serial_.transmit(sendASAP_buffer_, len + sizeof(generic_message_header_t)) == APP_HAL_OK) {
        ret = true;
    }
    return ret;
}

}  // namespace comm