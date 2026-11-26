#include "esp_bridge.h"
#include "driver/uart.h"
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/bk4819.h"
#include "driver/backlight.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "bsp/dp32g030/gpio.h"
#include "bsp/dp32g030/dma.h"
#include "misc.h"
#include "radio.h"
#include "audio.h"
#include <string.h>

// Протокол: [0xAA] [0x55] [CMD] [LEN_L] [LEN_H] [PAYLOAD...] [XOR_CRC]
#define SYNC1 0xAA
#define SYNC2 0x55

enum {
    CMD_PING          = 0x01,
    CMD_SCREEN_FULL   = 0x02, // 1024 байта буфера экрана (gFrameBuffer)
    CMD_SCREEN_LINE   = 0x03, // 1 байт номер строки (0..7) + 128 байт данных
    CMD_SET_FREQ      = 0x04, // uint32_t частота в 10 Гц
    CMD_WRITE_REG     = 0x05, // uint8_t reg + uint16_t val (прямой доступ к BK4819)
    CMD_READ_REG      = 0x06, // uint8_t reg -> ответ 2 байта
    CMD_SET_GPIO      = 0x07, // bit0: Backlight, bit1: Flashlight, bit2: AudioAmp, bit3: TX_PA
    CMD_SET_PTT       = 0x08, // 0 = RX, 1 = TX
    CMD_GET_TELEMETRY = 0x09, // Возвращает RSSI (2B), Noise (1B), Glitch (1B), KeyState (1B)
    EVT_KEY_CHANGE    = 0x81  // Рация сама шлет при нажатии/отпускании кнопки
};

static uint8_t rx_buf[1040];
static uint16_t rx_pos = 0;
static uint8_t rx_state = 0;
static uint16_t rx_len = 0;
static uint8_t rx_cmd = 0;
static uint8_t rx_crc = 0;

static void send_packet(uint8_t cmd, const uint8_t *data, uint16_t len) {
    uint8_t hdr[5] = {SYNC1, SYNC2, cmd, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t crc = cmd ^ hdr[3] ^ hdr[4];
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    UART_Send(hdr, 5);
    if (len > 0) UART_Send(data, len);
    UART_Send(&crc, 1);
}

static void handle_command(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    switch (cmd) {
        case CMD_PING: {
            uint8_t pong = 0x42;
            send_packet(CMD_PING, &pong, 1);
            break;
        }
        case CMD_SCREEN_FULL: {
            if (len == 1024) {
                memcpy(gFrameBuffer, payload, 1024);
                ST7565_BlitFullScreen();
            }
            break;
        }
        case CMD_SCREEN_LINE: {
            if (len == 129 && payload[0] < 8) {
                memcpy(gFrameBuffer[payload[0]], payload + 1, 128);
                ST7565_BlitLine(payload[0]);
            }
            break;
        }
        case CMD_SET_FREQ: {
            if (len == 4) {
                uint32_t freq = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
                BK4819_SetFrequency(freq);
                BK4819_PickRXFilterPathBasedOnFrequency(freq);
            }
            break;
        }
        case CMD_WRITE_REG: {
            if (len == 3) {
                uint16_t val = payload[1] | (payload[2] << 8);
                BK4819_WriteRegister(payload[0], val);
            }
            break;
        }
        case CMD_READ_REG: {
            if (len == 1) {
                uint16_t val = BK4819_ReadRegister(payload[0]);
                uint8_t resp[3] = {payload[0], (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
                send_packet(CMD_READ_REG, resp, 3);
            }
            break;
        }
        case CMD_SET_GPIO: {
            if (len >= 1) {
                uint8_t flags = payload[0];
                if (flags & 0x01) BACKLIGHT_TurnOn(); else BACKLIGHT_TurnOff();
                if (flags & 0x02) GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_FLASHLIGHT);
                else              GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_FLASHLIGHT);
                if (flags & 0x04) AUDIO_AudioPathOn(); else AUDIO_AudioPathOff();
            }
            break;
        }
        case CMD_SET_PTT: {
            if (len >= 1) {
                if (payload[0]) {
                    AUDIO_AudioPathOff();
                    BK4819_TurnsOffTones_TurnsOnRX();
                    BK4819_PrepareTransmit();
                    SYSTEM_DelayMs(10);
                    BK4819_TxOn_Beep();
                } else {
                    BK4819_TurnsOffTones_TurnsOnRX();
                    BK4819_SetupPowerAmplifier(0, 0);
                }
            }
            break;
        }
        case CMD_GET_TELEMETRY: {
            uint16_t rssi = BK4819_GetRSSI();
            uint8_t noise = BK4819_GetNoiseIndicator();
            uint8_t glitch = BK4819_GetGlitchIndicator();
            uint8_t key = (uint8_t)KEYBOARD_Poll();
            uint8_t resp[5] = {
                (uint8_t)(rssi & 0xFF), (uint8_t)(rssi >> 8),
                noise, glitch, key
            };
            send_packet(CMD_GET_TELEMETRY, resp, 5);
            break;
        }
    }
}

void ESP_Bridge_Run(void) {
    KEY_Code_t last_key = KEY_INVALID;
    bool last_ptt = false;
    uint16_t read_idx = 0;
    

        memset(gFrameBuffer, 0, sizeof(gFrameBuffer));
    ST7565_BlitFullScreen();

    while (1) {
        uint16_t write_idx = DMA_CH0->ST & 0xFFFU;
        if (read_idx == write_idx) break;
        uint8_t b = UART_DMA_Buffer[read_idx];
        read_idx = (read_idx + 1) % sizeof(UART_DMA_Buffer);
        
         
             
            switch (rx_state) {
                case 0: if (b == SYNC1) rx_state = 1; break;
                case 1: rx_state = (b == SYNC2) ? 2 : 0; break;
                case 2: rx_cmd = b; rx_crc = b; rx_state = 3; break;
                case 3: rx_len = b; rx_crc ^= b; rx_state = 4; break;
                case 4:
                    rx_len |= ((uint16_t)b << 8);
                    rx_crc ^= b;
                    rx_pos = 0;
                    if (rx_len > 1030) rx_state = 0;
                    else rx_state = (rx_len == 0) ? 6 : 5;
                    break;
                case 5:
                    rx_buf[rx_pos++] = b;
                    rx_crc ^= b;
                    if (rx_pos >= rx_len) rx_state = 6;
                    break;
                case 6:
                    if (b == rx_crc) {
                        handle_command(rx_cmd, rx_buf, rx_len);
                    }
                    rx_state = 0;
                    break;
            }
        }

        // 2. Проверяем нажатие кнопок и PTT
        KEY_Code_t key = KEYBOARD_Poll();
        bool ptt = !GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT);
        if (key != last_key || ptt != last_ptt) {
            last_key = key;
            last_ptt = ptt;
            uint8_t evt[2] = {(uint8_t)key, ptt ? 1 : 0};
            send_packet(EVT_KEY_CHANGE, evt, 2);
        }
    }
}
