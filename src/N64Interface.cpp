#include "N64Interface.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "rmt_gc_n64_encoder.h"

const static rmt_transmit_config_t n64_rmt_tx_config = {
    .loop_count = 0, // no transfer loop
    .flags = { 
        .eot_level = 1 // bus should be released in IDLE
    },
};

bool N64Interface::n64_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t task_woken = pdFALSE;
    N64Interface *handle = (N64Interface *)user_data;

    if(handle->_receiveHandler != nullptr)
    {
        task_woken = handle->_receiveHandler(edata, handle->_receiveHandlerUserData);
    }

    return task_woken;
}

size_t N64Interface::n64_rmt_decode_data(rmt_symbol_word_t *rmt_symbols, size_t symbol_num, uint8_t *decoded_bytes, size_t decoded_bytes_len)
{
    size_t byte_pos = 0, bit_pos = 0;
    for (size_t i = 0; i < symbol_num; i ++) {
        if(byte_pos > decoded_bytes_len - 1) {
            return byte_pos;
        }

        if (rmt_symbols[i].duration0 > 20) { // 0 bit
            decoded_bytes[byte_pos] &= ~(0x80 >> bit_pos); // MSB first
        } else { // 1 bit
            decoded_bytes[byte_pos] |= 0x80 >> bit_pos;
        }

        bit_pos ++;
        if (bit_pos >= 8) {
            bit_pos = 0;
            byte_pos ++;
        }
    }
    return byte_pos;
}

N64Interface::N64Interface(uint8_t pinData, size_t max_rx_bytes, ReceiveHandler recHandler, void *recUserData, rmt_receive_config_t rx_config, bool msbFirst)
{
    _data = static_cast<gpio_num_t>(pinData);
    _max_rx_bytes = max_rx_bytes;
    _msbFirst = msbFirst;
    _receiveHandler = recHandler;
    _receiveHandlerUserData = recUserData;
    _rmt_rx_config = rx_config;
}

N64Interface::~N64Interface() 
{
    if (_encoder) {
        rmt_del_encoder(_encoder);
    }
    if (_rx_channel) {
        rmt_disable(_rx_channel);
        rmt_del_channel(_rx_channel);
    }
    if (_tx_channel) {
        rmt_disable(_tx_channel);
        rmt_del_channel(_tx_channel);
    }
}

esp_err_t N64Interface::initialize()
{
    gc_n64_encoder_config_t encoder_config = {
        .resolution = N64_RMT_RESOLUTION_HZ,
        .msb_first = _msbFirst,
    };
    esp_err_t err = rmt_new_gc_n64_encoder(&encoder_config, &_encoder);
    if (err != ESP_OK)
        return err;

    rmt_rx_channel_config_t rx_channel_config = {
        .gpio_num = _data,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = N64_RMT_RESOLUTION_HZ, 
        .mem_block_symbols = _max_rx_bytes * 8, // Maximum bits to receive. (Read command returns 32 bytes + 1 stop bit.)
        .flags = {
            .invert_in = false,
            .with_dma = false,
            .io_loop_back = false,
        },
    };
    
    err = rmt_new_rx_channel(&rx_channel_config, &_rx_channel);
    if (err != ESP_OK)
        return err;

    rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = _data,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = N64_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .io_loop_back = true,
            .io_od_mode = true,
        },
    };
    
    err = rmt_new_tx_channel(&tx_channel_config, &_tx_channel);
    if (err != ESP_OK)
        return err;

    _rx_symbols = new rmt_symbol_word_t[_max_rx_bytes * 8];

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = n64_rmt_rx_done_callback,
    };
    err = rmt_rx_register_event_callbacks(_rx_channel, &cbs, this);
    if (err != ESP_OK)
        return err;

    rmt_enable(_tx_channel);
    rmt_enable(_rx_channel);

    return ESP_OK;
}

bool N64Interface::read(size_t bitsToReceive)
{
    esp_err_t err = rmt_receive(_rx_channel, _rx_symbols, bitsToReceive * sizeof(rmt_symbol_word_t), &_rmt_rx_config);
    if (err != ESP_OK) {
        ESP_LOGD("N64InterfaceBase", "Receive failed: %d", err);
        return false;
    }

    return true;
}

esp_err_t N64Interface::write(const uint8_t *send_buf, size_t len)
{
    return rmt_transmit(_tx_channel, _encoder, send_buf, len, &n64_rmt_tx_config);
}

bool N64Interface::waitForTx()
{
    esp_err_t err = rmt_tx_wait_all_done(_tx_channel, 10);
    if(err != ESP_OK)
    {
        ESP_LOGD("N64Interface", "TX wait failed %s", esp_err_to_name(err));
        return false;
    }
    return true;
}
