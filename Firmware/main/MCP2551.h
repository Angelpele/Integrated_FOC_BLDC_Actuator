//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//
#ifndef MCP2551_H
#define MCP2551_H

#include "ESP32_S3.h"


#define CAN_Speed 1  //Mbps
#define CHUNK_PAYLOAD 7
#define RX_TIMEOUT_MS 50


class CAN {
    public:
        int init();
        int test();
        bool sendData(TelemetryPacket* data);
        bool receiveData(TelemetryPacket* packet);

    private:
        void reset_rx_state();
        bool process_frame(const twai_message_t* msg);

        static uint8_t  rx_buf[sizeof(TelemetryPacket)];
        static uint8_t  chunks_received = 0;
        static uint8_t  expected_chunks = 0;
        static uint16_t chunks_mask     = 0;
        static uint32_t last_chunk_ms   = 0;
        static uint32_t packets_received = 0;
};

#endif