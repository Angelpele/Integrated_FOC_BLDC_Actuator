#include "driver/twai.h"

#define CAN_TX_PIN        GPIO_NUM_43
#define CAN_RX_PIN        GPIO_NUM_44
#define TELEMETRY_BASE_ID 0x100
#define MOTOR_ID          1
#define CHUNK_PAYLOAD     7
#define RX_TIMEOUT_MS     50

static uint16_t chunks_mask = 0;  // bit N set = chunk N received

#pragma pack(push, 1)
typedef struct {
    uint8_t  ID;
    float    pvdd;
    float    i1;
    float    i2;
    float    i3;
    float    temp;
    float    angle;
    float    red_angle;
    float    rpm;
    float    red_rpm;
    uint16_t A;
    uint16_t B;
    uint16_t C;
    uint8_t  nfault;
    uint8_t  oc;
    float    accx;
    float    accy;
    float    accz;
    float    gyrx;
    float    gyry;
    float    gyrz;
    uint8_t  ffall;
} TelemetryPacket;
#pragma pack(pop)

static uint8_t  rx_buf[sizeof(TelemetryPacket)];
static uint8_t  chunks_received = 0;
static uint8_t  expected_chunks = 0;
static uint32_t last_chunk_ms   = 0;
static bool     twai_installed  = false;
static uint32_t packets_received = 0;

void twai_setup() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return;
    if (twai_start() != ESP_OK) return;
    twai_installed = true;
}

void reset_rx_state() {
    memset(rx_buf, 0, sizeof(rx_buf));
    chunks_received = 0;
    expected_chunks = 0;
    chunks_mask     = 0;  // add this
}

bool process_frame(const twai_message_t* msg) {
    if (msg->data_length_code < 1) return false;

    uint8_t seq       = msg->data[0];
    uint8_t total     = (seq >> 4) & 0x0F;
    uint8_t chunk_idx = seq & 0x0F;

    // Timeout guard
    if (chunks_received > 0 && (millis() - last_chunk_ms) > RX_TIMEOUT_MS) {
        reset_rx_state();
    }

    if (chunk_idx == 0) {
        reset_rx_state();
        expected_chunks = total;
    } else if (expected_chunks == 0) {
        return false;
    }

    if (total != expected_chunks || chunk_idx >= total) {
        reset_rx_state();
        return false;
    }

    // Reject duplicate chunk
    if (chunks_mask & (1 << chunk_idx)) {
        return false;
    }

    size_t offset     = (size_t)chunk_idx * CHUNK_PAYLOAD;
    size_t pkt_size   = sizeof(TelemetryPacket);
    size_t bytes_left = (offset < pkt_size) ? pkt_size - offset : 0;
    size_t copy_len   = (bytes_left >= CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : bytes_left;

    if (copy_len > 0) {
        memcpy(rx_buf + offset, &msg->data[1], copy_len);
    }

    chunks_mask |= (1 << chunk_idx);
    chunks_received++;
    last_chunk_ms = millis();

    return (chunks_received == expected_chunks);
}

void rx_task(void* arg) {
    twai_message_t msg;
    while (true) {
        // Block indefinitely until a frame arrives — never misses a frame
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (process_frame(&msg)) {
                TelemetryPacket* pkt = (TelemetryPacket*)rx_buf;
                if (pkt->ID == MOTOR_ID) {
                    packets_received++;
                    // Print the entire packet to the Serial Monitor as fast as possible
                    Serial.printf("Pkt: %lu | ID:%d PVDD:%.2f I:[%.2f,%.2f,%.2f] T:%.2f Ang:%.2f RAng:%.2f RPM:%.2f RRPM:%.2f ABC:[%d,%d,%d] Flt:%d OC:%d Acc:[%.2f,%.2f,%.2f] Gyr:[%.2f,%.2f,%.2f] FF:%d\n",
                        packets_received, pkt->ID, pkt->pvdd, pkt->i1, pkt->i2, pkt->i3, pkt->temp, 
                        pkt->angle, pkt->red_angle, pkt->rpm, pkt->red_rpm, 
                        pkt->A, pkt->B, pkt->C, pkt->nfault, pkt->oc, 
                        pkt->accx, pkt->accy, pkt->accz, pkt->gyrx, pkt->gyry, pkt->gyrz, pkt->ffall);
                }
                reset_rx_state();
            }
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
    
    delay(200);
    twai_setup();
    reset_rx_state();

    // Pin to core 0, priority 5, 4KB stack
    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, NULL, 5, NULL, 0);
}

void loop() {
    // Only health check here — no receive polling
    twai_status_info_t s;
    twai_get_status_info(&s);
    if (s.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
        delay(100);
        twai_start();
    }
    delay(500);
}