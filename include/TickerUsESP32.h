#pragma once

/* This is the code of this library https://github.com/espressif/arduino-esp32/blob/master/libraries/Ticker/src/Ticker.h */
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_timer.h"
//    #include "driver/gptimer.h"
    // #include <stdio.h>
    //     // #include "freertos/semphr.h"
    // #include "driver/timer.h"
}

void system_timer_reinit();
#define INIT_US system_timer_reinit();

namespace TimersUS {
    class TickerUsESP32 {
    public:
    TickerUsESP32();
    ~TickerUsESP32();
    typedef void (*callback_t)();
    typedef void (*callback_with_arg_t)(void*);

    void setMaxIterations(uint32_t maxIterations) {
        _maxIterations = maxIterations;
    }

    void attach(uint32_t seconds, callback_t callback) {
        _attach_ms(seconds * 1000, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }
    void attach_ms(uint32_t milliseconds, callback_t callback) {
        _attach_ms(milliseconds, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }
    // Added delayed task
    void delay_ms(uint32_t milliseconds, callback_t callback) {
        _delay_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }
    // Added as uS
    void attach_us(uint64_t microseconds, callback_t callback) {
        _attach_us(microseconds, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }
    //test with timer.h
    // void test_us(uint32_t microseconds, timer_isr_t/*callback_with_arg_t*/ callback) {
    //     _test_us(microseconds, true, reinterpret_cast<timer_isr_t/*callback_with_arg_t*/>(callback), 0);
    // }
    // template<typename TArg>
    // void test_us(uint32_t microseconds, void (*callback)(TArg), TArg arg) {
    //     static_assert(sizeof(TArg) <= sizeof(uint32_t), "test_us() callback argument size must be <= 4 bytes");
    //     uint32_t arg32 = (uint32_t)arg;
    //     _test_us(microseconds, true, reinterpret_cast<timer_isr_t/*callback_with_arg_t*/>(callback), arg32);
    // }
    template<typename TArg>
    void attach(uint32_t seconds, void (*callback)(TArg), TArg arg) {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach() callback argument size must be <= 4 bytes");
        // C-cast serves two purposes:
        // static_cast for smaller integer types,
        // reinterpret_cast + const_cast for pointer types
        auto arg32 = (uint32_t)arg;
        _attach_ms(seconds * 1000, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }
    template<typename TArg>
    void attach_ms(uint32_t milliseconds, void (*callback)(TArg), TArg arg) {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_ms() callback argument size must be <= 4 bytes");
        auto arg32 = (uint32_t)arg;
        _attach_ms(milliseconds, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }
    // Added delayed task
    template<typename TArg>
    void delay_ms(uint32_t milliseconds, void (*callback)(TArg), TArg arg) {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "delay_ms() callback argument size must be <= 4 bytes");
        auto arg32 = (uint32_t)arg;
        _delay_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }
    // Added as uS
    template<typename TArg>
    void attach_us(uint64_t microseconds, void (*callback)(TArg), TArg arg) {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_us() callback argument size must be <= 4 bytes");
        auto arg32 = (uint32_t)arg;
        _attach_us(microseconds, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }

    // void once(float seconds, callback_t callback) {
    //     _attach_ms(seconds * 1000, false, reinterpret_cast<callback_with_arg_t>(callback), 0);
    // }

    // void once_ms(uint32_t milliseconds, callback_t callback) {
    //     _attach_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), 0);	
    // }

    // template<typename TArg>
    // void once(float seconds, void (*callback)(TArg), TArg arg) {
    //     static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach() callback argument size must be <= 4 bytes");
    //     uint32_t arg32 = (uint32_t)(arg);
    //     _attach_ms(seconds * 1000, false, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    // }

    // template<typename TArg>
    // void once_ms(uint32_t milliseconds, void (*callback)(TArg), TArg arg) {
    //     static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_ms() callback argument size must be <= 4 bytes");
    //     uint32_t arg32 = (uint32_t)(arg);
    //     _attach_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    // }

    void detach();
    bool active();

    private:
        uint32_t _iterationCount = 0;
        uint32_t _maxIterations = UINT32_MAX;  // Par défaut, pas de limite d'itérations

    protected:	
    void _attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg);
    // Added delayed task
    void _delay_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg);
    // Added as uS
    void _attach_us(uint64_t microseconds, bool repeat, callback_with_arg_t callback, uint32_t arg);

    // Using timer.h
//    void _test_us(int32_t microseconds, bool repeat, timer_isr_t/*callback_with_arg_t*/ callback, uint32_t arg);

    protected:
    esp_timer_handle_t _timer;
    esp_timer_handle_t _timer_delayed{};
//    gpt_timer_handle_t _gpttimer;
//    static SemaphoreHandle_t s_timer_sem;
    };
}

