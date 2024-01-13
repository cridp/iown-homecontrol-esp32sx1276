//#include <board-config.h>

#define USE_US_TIMER

#include "TickerUsESP32.h"
#include <thread>
#include <chrono>

#define TIMER_DIVIDER   (16)

namespace TimersUS {

    TickerUsESP32::TickerUsESP32() :
    _timer(nullptr) {}

    TickerUsESP32::~TickerUsESP32() {
    detach();
    }
    
    // void TickerUsESP32::_test_us(int32_t microseconds, bool repeat, timer_isr_t/*callback_with_arg_t*/ callback, uint32_t arg) {
    //     s_timer_sem = xSemaphoreCreateBinary();
    //     if (s_timer_sem == NULL) {
    //         printf("Binary semaphore can not be created");
    //     }
    //     timer_config_t config = {
    //         .alarm_en = TIMER_ALARM_EN,
    //         .counter_en = TIMER_PAUSE,
    //         .counter_dir = TIMER_COUNT_UP,
    //         .auto_reload = TIMER_AUTORELOAD_EN,
    //         .divider = TIMER_DIVIDER,
    //     };
    //     timer_init(TIMER_GROUP_0, TIMER_0, &config);
    //     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    //     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (TIMER_BASE_CLK / TIMER_DIVIDER));
    //     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    //     timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, callback, reinterpret_cast<void*>(arg)/*NULL*/, 0);
    //     timer_start(TIMER_GROUP_0, TIMER_0);
    // }

    void TickerUsESP32::_attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
    // Attendre le délai
//    const uint32_t delay_microseconds = 100; // * 1000;
//    vTaskDelay(milliseconds); //delay_microseconds / portTICK_PERIOD_MS);

    esp_timer_create_args_t _timerConfig;
    _timerConfig.arg = reinterpret_cast<void*>(arg);
    _timerConfig.callback = callback;
    _timerConfig.dispatch_method = ESP_TIMER_TASK; //ESP_TIMER_ISR; //
    _timerConfig.skip_unhandled_events = false; //true;
    _timerConfig.name = "TickerMsESP32";
    if (_timer) {
        ESP_ERROR_CHECK(esp_timer_stop(_timer));
        ESP_ERROR_CHECK(esp_timer_delete(_timer));
    }
    ESP_ERROR_CHECK(esp_timer_create(&_timerConfig, &_timer));
    if (repeat) {
//        printf("Packet %u Periodic\n", milliseconds);
        ESP_ERROR_CHECK(esp_timer_start_periodic(_timer, milliseconds * 1000ULL));
    } else {
        ESP_ERROR_CHECK(esp_timer_start_once(_timer, milliseconds * 1000ULL));
    }
    }
    // Added delayed task
   void TickerUsESP32::_delay_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
    // Attendre le délai
    // const uint32_t delay_microseconds = 100; // * 1000;
    // vTaskDelay(milliseconds); //delay_microseconds / portTICK_PERIOD_MS);
//    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));

    esp_timer_create_args_t _timerConfig;
    _timerConfig.arg = reinterpret_cast<void*>(arg);
    _timerConfig.callback = callback;
    _timerConfig.dispatch_method = ESP_TIMER_TASK; //ESP_TIMER_ISR; //
//    _timerConfig.skip_unhandled_events = true;
    _timerConfig.name = "TickerMsESP32Delay";
    if (_timer_delayed) {
//        ESP_ERROR_CHECK(esp_timer_stop(_timer_delayed));
        ESP_ERROR_CHECK(esp_timer_delete(_timer_delayed));
    }
    ESP_ERROR_CHECK(esp_timer_create(&_timerConfig, &_timer_delayed));
//    if (repeat) {
//        ESP_ERROR_CHECK(esp_timer_start_periodic(_timer, milliseconds * 1000ULL));
//    } else {
 //       std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
//        printf("Packet %u Delayed\n", milliseconds);
        ESP_ERROR_CHECK(esp_timer_start_once(_timer_delayed, milliseconds * 1000ULL));
//    }
    }

    // Added as uS
    void TickerUsESP32::_attach_us(uint64_t microseconds, bool repeat, callback_with_arg_t callback, uint32_t arg) {
    esp_timer_create_args_t _timerConfig;
    _timerConfig.arg = reinterpret_cast<void*>(arg);
    _timerConfig.callback = callback;
    _timerConfig.dispatch_method = ESP_TIMER_TASK; //ESP_TIMER_ISR; // But ISR doesnt work with 100ULL  To see if it work with MQTT and WIFI. Can set callback in IRAM
//    _timerConfig.skip_unhandled_events = true;
    _timerConfig.name = "TickerUsESP32";
    
    if (_timer) {
        ESP_ERROR_CHECK(esp_timer_stop(_timer));
        ESP_ERROR_CHECK(esp_timer_delete(_timer));
    }

    ESP_ERROR_CHECK(esp_timer_create(&_timerConfig, &_timer));
    if (repeat) {
        ESP_ERROR_CHECK(esp_timer_start_periodic(_timer, microseconds));
    } else {
        ESP_ERROR_CHECK(esp_timer_start_once(_timer, microseconds));
    }
    }

    void TickerUsESP32::detach() {
    if (_timer) {
        ESP_ERROR_CHECK(esp_timer_stop(_timer));
        ESP_ERROR_CHECK(esp_timer_delete(_timer));
        _timer = nullptr;
    }
    //     if (_timer_delayed) {
    //     ESP_ERROR_CHECK(esp_timer_stop(_timer_delayed));
    //     ESP_ERROR_CHECK(esp_timer_delete(_timer_delayed));
    //     _timer = nullptr;
    // }
    }

    bool TickerUsESP32::active() {
    if (!_timer) return false;
    return esp_timer_is_active(_timer);
    }
}
//#endif