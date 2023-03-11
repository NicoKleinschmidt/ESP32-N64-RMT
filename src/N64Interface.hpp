#pragma once

#include "stdint.h"
#include "driver/gpio.h"
#include "driver/rmt_common.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

typedef bool (*ReceiveHandler)(const rmt_rx_done_event_data_t *edata, void *user_data);

class N64Interface
{
private:
    rmt_channel_handle_t _rx_channel;
    rmt_channel_handle_t _tx_channel;
    rmt_encoder_handle_t _encoder;
    rmt_symbol_word_t *_rx_symbols;

    ReceiveHandler _receiveHandler;
    void *_receiveHandlerUserData;

    size_t _max_rx_bytes;
    gpio_num_t _data;
    bool _msbFirst;

    static bool n64_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

public:
    N64Interface(uint8_t pinData, size_t max_rx_bytes, ReceiveHandler recHandler, void *recUserData, bool _msbFirst = true);
    ~N64Interface();

    static size_t n64_rmt_decode_data(rmt_symbol_word_t *rmt_symbols, size_t symbol_num, uint8_t *decoded_bytes, size_t decoded_bytes_len);
    
    esp_err_t initialize();

    bool read(size_t bitsToRead);
    esp_err_t write(const uint8_t *send_buf, size_t len);
};
