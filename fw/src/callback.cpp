#include "callback.hpp"

Callback::Callback(callback_t callback, callback_arg_t arg)
    : m_callback(callback),
      m_callback_arg(arg)
{}

void Callback::operator ()() {
    if (m_callback)
        m_callback(m_callback_arg);
}

const Callback::callback_t&Callback:: getCallback() const { return m_callback; }

const Callback::callback_arg_t& Callback::getArg() const { return m_callback_arg; }
