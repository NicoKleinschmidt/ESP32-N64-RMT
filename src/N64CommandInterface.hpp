#pragma once

#include "stdint.h"
#include "driver/gpio.h"
#include "driver/rmt_common.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "N64Interface.hpp"

class N64CommandInterface
{
private:
    N64Interface *_interface;
    QueueHandle_t _receiveQueue;

    static bool receiveHandler(const rmt_rx_done_event_data_t *edata, void *user_data);

public:
    N64CommandInterface(uint8_t pinData, size_t max_rx_bytes, bool _msbFirst = true);
    ~N64CommandInterface();

    esp_err_t initialize();
    
    size_t send_command(uint8_t *command, size_t command_len, uint8_t *receiveBuffer, size_t receiveBufferLen);
};
