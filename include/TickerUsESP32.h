#pragma once

extern "C" {
  #include "esp_timer.h"
}

void system_timer_reinit();
#define INIT_US system_timer_reinit();

//namespace Timers
//{
    class TickerUsESP32
    {
    public:
    TickerUsESP32();
    ~TickerUsESP32();
    typedef void (*callback_t)(void);
    typedef void (*callback_with_arg_t)(void*);

    void attach(float seconds, callback_t callback)
    {
        _attach_ms(seconds * 1000, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }

    void attach_ms(uint32_t milliseconds, callback_t callback)
    {
        _attach_ms(milliseconds, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }


    // Added as uS
    void attach_us(uint32_t microseconds, callback_t callback)
    {
        _attach_us(microseconds, true, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }



    template<typename TArg>
    void attach(float seconds, void (*callback)(TArg), TArg arg)
    {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach() callback argument size must be <= 4 bytes");
        // C-cast serves two purposes:
        // static_cast for smaller integer types,
        // reinterpret_cast + const_cast for pointer types
        uint32_t arg32 = (uint32_t)arg;
        _attach_ms(seconds * 1000, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }

    template<typename TArg>
    void attach_ms(uint32_t milliseconds, void (*callback)(TArg), TArg arg)
    {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_ms() callback argument size must be <= 4 bytes");
        uint32_t arg32 = (uint32_t)arg;
        _attach_ms(milliseconds, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }


    // Added as uS
    template<typename TArg>
    void attach_us(uint32_t microseconds, void (*callback)(TArg), TArg arg)
    {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_us() callback argument size must be <= 4 bytes");
        uint32_t arg32 = (uint32_t)arg;
        _attach_ms(microseconds, true, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }


    void once(float seconds, callback_t callback)
    {
        _attach_ms(seconds * 1000, false, reinterpret_cast<callback_with_arg_t>(callback), 0);
    }

    void once_ms(uint32_t milliseconds, callback_t callback)
    {
        _attach_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), 0);	
    }

    template<typename TArg>
    void once(float seconds, void (*callback)(TArg), TArg arg)
    {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach() callback argument size must be <= 4 bytes");
        uint32_t arg32 = (uint32_t)(arg);
        _attach_ms(seconds * 1000, false, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }

    template<typename TArg>
    void once_ms(uint32_t milliseconds, void (*callback)(TArg), TArg arg)
    {
        static_assert(sizeof(TArg) <= sizeof(uint32_t), "attach_ms() callback argument size must be <= 4 bytes");
        uint32_t arg32 = (uint32_t)(arg);
        _attach_ms(milliseconds, false, reinterpret_cast<callback_with_arg_t>(callback), arg32);
    }

    void detach();
    bool active();

    protected:	
    void _attach_ms(uint32_t milliseconds, bool repeat, callback_with_arg_t callback, uint32_t arg);

    // Added as uS
    void _attach_us(uint32_t microseconds, bool repeat, callback_with_arg_t callback, uint32_t arg);


    protected:
    esp_timer_handle_t _timer;
    };
//}

