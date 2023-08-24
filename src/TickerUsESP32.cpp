#define USE_US_TIMER

#if defined(ESP32)

#include "TickerUsESP32.h"

//namespace Timers
//{

    TickerUsESP32::TickerUsESP32() :
    _timer(nullptr) {}

    TickerUsESP32::~TickerUsESP32() {
    detach();
    }

    void TickerUsESP32::_attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
    esp_timer_create_args_t _timerConfig;
    _timerConfig.arg = reinterpret_cast<void*>(arg);
    _timerConfig.callback = callback;
    _timerConfig.dispatch_method = ESP_TIMER_TASK;
    _timerConfig.name = "TickerUsESP32";
    if (_timer) {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
    }
    esp_timer_create(&_timerConfig, &_timer);
    if (repeat) {
        esp_timer_start_periodic(_timer, milliseconds * 1000ULL);
    } else {
        esp_timer_start_once(_timer, milliseconds * 1000ULL);
    }
    }


    // Added as uS
    void TickerUsESP32::_attach_us(uint32_t microseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
    esp_timer_create_args_t _timerConfig;
    _timerConfig.arg = reinterpret_cast<void*>(arg);
    _timerConfig.callback = callback;
    _timerConfig.dispatch_method = ESP_TIMER_TASK;
    _timerConfig.name = "TickerUsESP32";
    if (_timer) {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
    }
    esp_timer_create(&_timerConfig, &_timer);
    if (repeat) {
        esp_timer_start_periodic(_timer, microseconds);
    } else {
        esp_timer_start_once(_timer, microseconds);
    }
    }



    void TickerUsESP32::detach() {
    if (_timer) {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
        _timer = nullptr;
    }
    }

    bool TickerUsESP32::active() {
    if (!_timer) return false;
    return esp_timer_is_active(_timer);
    }
//}
#endif