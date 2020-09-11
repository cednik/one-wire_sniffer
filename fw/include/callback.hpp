#pragma once

#include <esp_attr.h>

class Callback {
public:
    typedef void* callback_arg_t;
    typedef void (*callback_t)(callback_arg_t);
    
    Callback(callback_t callback = nullptr, callback_arg_t arg = nullptr);
    void INTR_ATTR operator ()();
    const callback_t& getCallback() const;
    const callback_arg_t& getArg() const;
private:
    callback_t m_callback;
    callback_arg_t m_callback_arg;
};
