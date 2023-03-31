#include "N64CommandInterface.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "rmt_gc_n64_encoder.h"

const static rmt_receive_config_t n64_rmt_rx_config = {
    .signal_range_min_ns = 1000000000 / N64_RMT_RESOLUTION_HZ,
    .signal_range_max_ns = 18 * 1000,
};

bool N64CommandInterface::receiveHandler(const rmt_rx_done_event_data_t *edata, void *user_data)
{
    N64CommandInterface *handle = (N64CommandInterface *)user_data;
    BaseType_t task_woken = pdFALSE;

    xQueueSendFromISR(handle->_receiveQueue, edata, &task_woken);

    return task_woken;
}

N64CommandInterface::N64CommandInterface(uint8_t pinData, size_t max_rx_bytes, bool msbFirst)
{
    _interface = new N64Interface(pinData, max_rx_bytes, receiveHandler, this, n64_rmt_rx_config, msbFirst);
    _receiveQueue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
}

N64CommandInterface::~N64CommandInterface() 
{
    if(_interface != nullptr) {
        delete _interface;
    }
    if(_receiveQueue != nullptr) {
        vQueueDelete(_receiveQueue);
    }
}

esp_err_t N64CommandInterface::initialize()
{
    return _interface->initialize();
}

size_t N64CommandInterface::send_command(uint8_t *command, size_t command_len, uint8_t *receiveBuffer, size_t receiveBufferLen)
{
    size_t bitsToReceive = (receiveBufferLen + command_len) * 8 + 2; // Data we send + data we expect + stop bits for send and receive.

    if(!_interface->read(bitsToReceive)) {
        return 0;
    }
    esp_err_t err = _interface->write(command, command_len);
    if (err != ESP_OK) {
        ESP_LOGD("N64Interface", "Transmit failed: %d", err);
        return 0;
    }

    rmt_rx_done_event_data_t rmt_rx_evt_data;
    if (xQueueReceive(_receiveQueue, &rmt_rx_evt_data, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGD("N64Interface", "Timeout");
        return 0;
    }

    if(rmt_rx_evt_data.num_symbols < command_len * 8 + 2)
    {
        ESP_LOGD("N64Interface", "Only received %d symbols. Min: %d", rmt_rx_evt_data.num_symbols, command_len * 8 + 2);
        return 0;
    }

    size_t receivedDataLenBits = rmt_rx_evt_data.num_symbols - command_len * 8 - 2;
    return N64Interface::n64_rmt_decode_data(rmt_rx_evt_data.received_symbols + command_len * 8 + 1, receivedDataLenBits, receiveBuffer, receiveBufferLen);
}
