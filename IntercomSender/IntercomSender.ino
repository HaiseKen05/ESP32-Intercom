#include <esp_now.h>
#include <WiFi.h>
#include <ESP_I2S.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============================================================
//  REPLACE WITH THE MAC ADDRESS OF THE OTHER DEVICE
// ============================================================
uint8_t peerAddress[] = {0xCC,0xDB,0xA7,0x99,0x70,0xC8};

// ============================================================
//  PIN DEFINITIONS  (same wiring on both devices)
// ============================================================
// ---- INMP441 (Input) ----
#define MIC_SCK 26
#define MIC_WS  25
#define MIC_SD  32

// ---- MAX98357A (Output) ----
#define AMP_BCLK 14
#define AMP_LRC  27
#define AMP_DIN  22

// ---- Push-to-Talk Button ----
#define BUTTON_PIN 4

// ============================================================
//  AUDIO CONSTANTS
// ============================================================
const int   SAMPLE_RATE        = 16000;
const int   MIC_BUFFER_SAMPLES = 128;   // I2S DMA read size
const int   SAMPLES_PER_PACKET = 120;   // 120 * 2 = 240 bytes payload
const float GAIN               = 0.1f;

// ============================================================
//  ESP-NOW AUDIO PACKET  (2 + 240 = 242 bytes)
// ============================================================
typedef struct struct_audio_packet {
    uint16_t seq;
    int16_t  samples[SAMPLES_PER_PACKET];
} struct_audio_packet;

struct_audio_packet txPacket;
struct_audio_packet rxPacket;

// ============================================================
//  GLOBALS
// ============================================================
I2SClass I2S_In;
I2SClass I2S_Out;

int32_t micBuffer[MIC_BUFFER_SAMPLES];

volatile bool buttonPressed   = false;
volatile bool lastButtonState = false;

uint16_t txSeq = 0;

// ============================================================
//  ESP-NOW SEND CALLBACK  (Core V3 signature)
// ============================================================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Do NOT print at full rate — only failures.
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("TX Fail");
    }
}

// ============================================================
//  ESP-NOW RECEIVE CALLBACK  (Core V3 signature)
//  Writes incoming audio straight to the MAX98357A.
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data, int len) {
    if (len != sizeof(struct_audio_packet)) return;

    memcpy(&rxPacket, data, sizeof(rxPacket));

    // Push 240 bytes of PCM into the amp's I2S DMA.
    I2S_Out.write((uint8_t*)rxPacket.samples,
                  sizeof(rxPacket.samples));
}

// ============================================================
//  MIC -> ESP-NOW TASK
//  Runs on its own core so the receive callback is never starved.
// ============================================================
void micTxTask(void *param) {
    for (;;) {
        // ---- Always drain the mic DMA ----
        size_t bytesRead = I2S_In.readBytes((char*)micBuffer,
                                            sizeof(micBuffer));
        int samplesRead = bytesRead / 4;
        if (samplesRead == 0) continue;

        int samplesToSend = (samplesRead < SAMPLES_PER_PACKET)
                            ? samplesRead
                            : SAMPLES_PER_PACKET;

        // ---- Convert 32-bit -> 16-bit with gain, or send silence ----
        if (buttonPressed) {
            for (int i = 0; i < samplesToSend; i++) {
                int32_t s = micBuffer[i] >> 14;
                s = (int32_t)(s * GAIN);
                if (s >  32767) s =  32767;
                if (s < -32768) s = -32768;
                txPacket.samples[i] = (int16_t)s;
            }
        } else {
            for (int i = 0; i < samplesToSend; i++) {
                txPacket.samples[i] = 0;
            }
        }

        // Zero-fill any remainder if we read fewer than 120 samples.
        for (int i = samplesToSend; i < SAMPLES_PER_PACKET; i++) {
            txPacket.samples[i] = 0;
        }

        txPacket.seq = txSeq++;

        esp_now_send(peerAddress,
                     (uint8_t*)&txPacket,
                     sizeof(txPacket));
    }
}

// ============================================================
//  BUTTON TASK  (kept out of the audio task to avoid blocking)
// ============================================================
void buttonTask(void *param) {
    for (;;) {
        bool current = (digitalRead(BUTTON_PIN) == HIGH);
        if (current != lastButtonState) {
            Serial.println(current ? ">> TALKING" : ">> MUTED");
            lastButtonState = current;
        }
        buttonPressed = current;
        vTaskDelay(pdMS_TO_TICKS(10));   // light debounce / pacing
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP-NOW Full-Duplex Audio Starting...");

    // ---- Button ----
    pinMode(BUTTON_PIN, INPUT);   // external 10k pull-down

    // ---- Wi-Fi Station mode ----
    WiFi.mode(WIFI_STA);
    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());

    // ---- ESP-NOW ----
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // ---- I2S Input (INMP441) ----
    I2S_In.setPins(MIC_SCK, MIC_WS, -1, MIC_SD, -1);
    if (!I2S_In.begin(I2S_MODE_STD, SAMPLE_RATE,
                      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("Failed to init I2S Input!");
        while (1);
    }

    // ---- I2S Output (MAX98357A) ----
    I2S_Out.setPins(AMP_BCLK, AMP_LRC, AMP_DIN, -1, -1);
    if (!I2S_Out.begin(I2S_MODE_STD, SAMPLE_RATE,
                       I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("Failed to init I2S Output!");
        while (1);
    }

    // ---- Pin the audio task to core 1, button task to core 0 ----
    xTaskCreatePinnedToCore(micTxTask,  "micTxTask",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 1, NULL, 0);

    Serial.println("Ready. Hold the button to talk.");
}

// ============================================================
//  LOOP  (unused — all work is in tasks/callbacks)
// ============================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}