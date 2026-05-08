#include "driver/twai.h"

#define CAN_TX_PIN        GPIO_NUM_43
#define CAN_RX_PIN        GPIO_NUM_44
#define TELEMETRY_BASE_ID 0x100
#define MOTOR_ID          1
#define CHUNK_PAYLOAD     7

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

static bool twai_installed = false;

void twai_setup() {
    // Normal mode — requires ACK from the other node
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return;
    if (twai_start() != ESP_OK) return;
    twai_installed = true;
}

bool can_send_telemetry(const TelemetryPacket* pkt) {
    if (!twai_installed) return false;

    const uint8_t* raw = (const uint8_t*)pkt;
    size_t total       = sizeof(TelemetryPacket);
    uint8_t num_chunks = (total + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;

    for (uint8_t i = 0; i < num_chunks; i++) {
        twai_message_t msg = {};
        msg.identifier       = TELEMETRY_BASE_ID | MOTOR_ID;
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

        if (twai_transmit(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            return false;
        }
        delayMicroseconds(2000);
    }
    return true;
}

TelemetryPacket demo_packet = {
    .ID        = MOTOR_ID,
    .pvdd      = 12.5f,
    .i1        = 1.23f,
    .i2        = 1.10f,
    .i3        = -2.33f,
    .temp      = 42.0f,
    .angle     = 0.0f,
    .red_angle = 0.0f,
    .rpm       = 1500.0f,
    .red_rpm   = 100.0f,
    .A         = 2048,
    .B         = 2060,
    .C         = 2035,
    .nfault    = 1,
    .oc        = 0,
    .accx      = 0.01f,
    .accy      = 0.00f,
    .accz      = 9.81f,
    .gyrx      = 0.0f,
    .gyry      = 0.0f,
    .gyrz      = 0.0f,
    .ffall     = 0,
};

uint32_t tx_count = 0;

void setup() {
    delay(200);
    twai_setup();
}

void loop() {
    demo_packet.angle += 1.0f;
    if (demo_packet.angle >= 360.0f) demo_packet.angle = 0.0f;

    bool ok = can_send_telemetry(&demo_packet);
    tx_count++;

    // Print status every 50 packets via TWAI status
    if (tx_count % 50 == 0) {
        twai_status_info_t s;
        twai_get_status_info(&s);
        // state=1 running, txerr=0 good, if txerr climbs the receiver is not ACKing
        // You can probe this with a logic analyzer or LED
        // LED on GPIO2: solid = OK, fast blink = error
        if (s.state != TWAI_STATE_RUNNING || s.tx_error_counter > 0) {
            // Signal error — fast blink on any free LED pin
            for (int i = 0; i < 10; i++) {
                gpio_set_level(GPIO_NUM_2, 1); delay(50);
                gpio_set_level(GPIO_NUM_2, 0); delay(50);
            }
        }
    }

    delay(50); // 20 Hz
}