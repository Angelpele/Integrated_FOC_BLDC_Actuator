//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//

#include "MCP2551.h"

int CAN::init()
{
    gpio_set_direction(CAN_RX, GPIO_MODE_INPUT);
    gpio_set_direction(CAN_TX, GPIO_MODE_OUTPUT); 
        
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();      // Change filter to accept specific IDs

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        printf("CAN INIT: Failed to install TWAI driver\n");
        return -1;
    }
    if (twai_start() != ESP_OK) {
        printf("CAN INIT: Failed to start TWAI driver\n");
        return -1;
    }
    return 0;
}

int CAN::test()
{
    TelemetryPacket test = {};
    test.ID   = MOTOR_ID;
    test.pvdd = 10.0f;   
    test.rpm  = 1234.0f;

    // Send dummy packet
    if (!sendData(&test)) {
        printf("CAN TEST: TX failed\n");
        return -1;
    }
    return 0;
}

bool CAN::sendData(TelemetryPacket* data)
{
    const uint8_t* raw = (const uint8_t*)data;
    size_t total = sizeof(TelemetryPacket);
    uint8_t num_chunks = (total + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;

    for (uint8_t i = 0; i < num_chunks; i++) {
        twai_message_t msg = {};
        msg.identifier       = MOTOR_ID;
        msg.extd             = 0;
        msg.rtr              = 0;
        msg.dlc_non_comp     = 0;
        msg.data_length_code = 8;

        size_t offset    = (size_t)i * CHUNK_PAYLOAD;
        size_t remaining = total - offset;
        size_t chunk_len = (remaining >= CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : remaining;

        msg.data[0] = ((num_chunks & 0x0F) << 4) | (i & 0x0F);
        memset(&msg.data[1], 0, CHUNK_PAYLOAD);
        memcpy(&msg.data[1], raw + offset, chunk_len);

        if (twai_transmit(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {return false;}
        delayMicroseconds(2000);
    }
    return true;
}

// Clean up and reset state for new packet reception
void reset_rx_state() {
    memset(rx_buf, 0, sizeof(rx_buf));
    chunks_received = 0;
    expected_chunks = 0;
    chunks_mask     = 0;  
}


bool process_frame(const twai_message_t* msg)
{
    uint8_t seq       = msg->data[0];
    uint8_t total     = (seq >> 4) & 0x0F;
    uint8_t chunk_idx = seq & 0x0F;

    // Reset received data if timeout since last chunk
    if (chunks_received > 0 && (millis() - last_chunk_ms) > RX_TIMEOUT_MS)
        reset_rx_state();

    // chunk 0 always starts a new packet
    if (chunk_idx == 0) {
        reset_rx_state();
        expected_chunks = total;
    } else if (expected_chunks == 0) {
        return false; // missed chunk 0, wait for next burst
    }

    // Discard if sequence is inconsistent
    if (total != expected_chunks || chunk_idx >= total)  { reset_rx_state(); return false; }
    if (chunks_mask & (1 << chunk_idx))                    return false;

    // Copy payload into buffer
    size_t offset    = (size_t)chunk_idx * CHUNK_PAYLOAD;
    size_t copy_len  = ((sizeof(TelemetryPacket) - offset) >= CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : (sizeof(TelemetryPacket) - offset);
    memcpy(rx_buf + offset, &msg->data[1], copy_len);

    chunks_mask |= (1 << chunk_idx);
    chunks_received++;
    last_chunk_ms = millis();

    return (chunks_received == expected_chunks);    // Return true if packet is complete
}

bool receiveData(TelemetryPacket* packet) {
    twai_message_t msg;
    while (true) {
        // Block indefinitely until a frame arrives — never misses a frame
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (process_frame(&msg)) {
                TelemetryPacket* pkt = (TelemetryPacket*)rx_buf;
                if (pkt->ID == MOTOR_ID) {
                    packets_received++;
                    *packet = *pkt;
                    return true;
                }
                reset_rx_state();
            }
        }
    }
}