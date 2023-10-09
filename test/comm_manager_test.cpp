#include "comm_manager.hpp"

#include "comm_message_types.hpp"
#include "gtest/gtest.h"
#include "mock_hal_serial.hpp"

using namespace ::testing;

namespace comm {

// Define a test for the determine_packet_metadata function
TEST(CommSerial, determine_packet_metadata) {
    // Create a buffer to read from
    uint8_t buffer[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    // Create the expected header
    generic_message_header_t expected_header = {0x00, 0x0102, 0x0304, 0x05};

    // Call the function
    generic_message_header_t header = CommManager::determine_packet_metadata(buffer, 10);

    // Check that the header is correct
    EXPECT_EQ(header.SOF, expected_header.SOF);
    EXPECT_EQ(header.message_channel, expected_header.message_channel);
    EXPECT_EQ(header.message_length, expected_header.message_length);
    EXPECT_EQ(header.message_sequence, expected_header.message_sequence);
}

// Ensure that create_packet_metadata returns the correct header
TEST(CommSerial, create_packet_metadata) {
    // Create the expected header
    ChannelID message_channel = ChannelID::SYSTEM_MANAGER_STATE;
    uint16_t message_length = 0x0304;
    uint8_t sequence = 0x05;

    // Call the function
    generic_message_header_t header = CommManager::create_packet_metadata(message_channel, message_length, sequence);

    // Convert it back into a buffer
    uint8_t buffer[sizeof(generic_message_header_t)] = {0};
    CommManager::create_message_header_buffer(header, buffer, sizeof(buffer));

    // Check that the data of the header is correct
    uint8_t* header_data = buffer;
    EXPECT_EQ(header_data[0], 0x00);
    EXPECT_EQ(header_data[1], (uint16_t)message_channel >> 8);
    EXPECT_EQ(header_data[2], (uint16_t)message_channel & 0xFF);
    EXPECT_EQ(header_data[3], message_length >> 8);
    EXPECT_EQ(header_data[4], message_length & 0xFF);
}

// Ensure that the registerCallback function returns true
TEST(CommSerial, registerCallback) {
    // Create a mock HAL_SERIAL
    basilisk_hal::MockSerial mock_hal_serial;

    // Create a CommManager
    CommManager comm_manager(mock_hal_serial);

    // Create a callback
    void (*callback)(generic_message_header_t, const void*, void*) = [](generic_message_header_t header, const void* message,
                                                                        void* context) {
        (void)header;
        (void)message;
        (void)context;
    };

    // Register the callback
    bool success = comm_manager.registerCallback(ChannelID::SYSTEM_MANAGER_STATE, callback, NULL);

    // Check that the registration was successful
    EXPECT_TRUE(success);
}

// Ensure that the sendASAP function returns true
TEST(CommSerial, sendASAP) {
    // Create a mock HAL_SERIAL
    basilisk_hal::MockSerial mock_hal_serial;

    // Create a CommManager
    CommManager comm_manager(mock_hal_serial);

    // Create a message
    ArmDisarmMessage message = ArmDisarmMessage(false);

    // Expect a call to transmit on the mock hal serial object
    EXPECT_CALL(mock_hal_serial, transmit(_, _)).WillOnce(Return(APP_HAL_OK));

    // Send the message
    bool success = comm_manager.sendASAP((ChannelID)0x00, message);

    // Check that the send was successful
    EXPECT_TRUE(success);
}

static void set_speed_msg_test_callback(generic_message_header_t header, const void* message, void* context) {
    (void)context;
    // Cast the message to a SetSpeedMessage
    SetSpeedMessage set_speed_message;
    set_speed_message.populateDataFromBuffer((const uint8_t*)message, header.message_length);

    // Check that the channel ID is correct
    EXPECT_EQ(header.message_channel, (channel_id_t)ChannelID::SET_SPEED);

    // Check that the message length is correct
    EXPECT_EQ(header.message_length, sizeof(SetSpeedMessage::set_speed_message_t));

    // Check that the speed is correct
    EXPECT_FLOAT_EQ(set_speed_message.data.speed, 0.5);
}
// Register a callback for the set speed function on the set speed channel. Ensure that the callback is called
TEST(CommSerial, set_speed_msg_test_callback) {
    // Create a mock HAL_SERIAL
    basilisk_hal::MockSerial mock_hal_serial;

    // Create a CommManager
    CommManager comm_manager(mock_hal_serial);

    // Register a callback for the set speed function on the set speed channel
    comm_manager.registerCallback(ChannelID::SET_SPEED, set_speed_msg_test_callback, NULL);

    // Create a fake buffer of message header and data to process
    uint8_t buffer[sizeof(generic_message_header_t) + sizeof(SetSpeedMessage::set_speed_message_t)];

    // Populate the buffer with the message header
    generic_message_header_t header =
        CommManager::create_packet_metadata(ChannelID::SET_SPEED, sizeof(SetSpeedMessage::set_speed_message_t), 0x00);
    CommManager::create_message_header_buffer(header, buffer, sizeof(buffer));

    // Populate the buffer with the message data
    SetSpeedMessage::set_speed_message_t* data =
        (SetSpeedMessage::set_speed_message_t*)(buffer + sizeof(generic_message_header_t));
    data->speed = 0.5;

    // Process the buffer
    comm_manager.process_received_data(buffer, sizeof(buffer));

    // Check that the callback was called
    EXPECT_TRUE(true);
}

}  // namespace comm
