#pragma once

template <typename T, size_t Capacity>
class Buffer {
public:
    typedef size_t index_type;
    typedef T value_type;
    static const index_type capacity = Capacity;

    Buffer()
        : /*m_lock(portMUX_INITIALIZER_UNLOCKED),*/
          m_rptr(0),
          m_wptr(0)
    {}

    void INTR_ATTR clear() { m_rptr = m_wptr; }

    bool INTR_ATTR empty() const { return m_wptr == m_rptr; }

    bool INTR_ATTR full() const { return next(m_wptr) == m_rptr; }

    bool INTR_ATTR push(value_type v) {
        //portENTER_CRITICAL_ISR(&m_lock);
        index_type wptr = m_wptr;
        m_buffer[wptr] = v;
        m_wptr = next(wptr);
        bool overflow = m_wptr != m_rptr;
        //portEXIT_CRITICAL_ISR(&m_lock);
        return overflow;
    }

    value_type INTR_ATTR top() const { return m_buffer[m_rptr]; }

    volatile value_type const & INTR_ATTR top_ref() const { return m_buffer[m_rptr]; }

    volatile value_type & INTR_ATTR top_ref() { return m_buffer[m_rptr]; }

    value_type INTR_ATTR first() const { return m_buffer[m_rptr]; }

    volatile value_type const & INTR_ATTR first_ref() const { return m_buffer[m_rptr]; }

    volatile value_type & INTR_ATTR first_ref() { return m_buffer[m_rptr]; }

    value_type INTR_ATTR last() const { return m_buffer[prev(m_wptr)]; }

    volatile value_type const & INTR_ATTR last_ref() const { return m_buffer[prev(m_wptr)]; }

    volatile value_type & INTR_ATTR last_ref() { return m_buffer[prev(m_wptr)]; }

    index_type INTR_ATTR size() const { return dist(m_wptr, m_rptr); }

    value_type INTR_ATTR operator[] (index_type i) const { return m_buffer[next(m_rptr, i)]; }

    void INTR_ATTR pop() {
        //portENTER_CRITICAL_ISR(&m_lock);
        m_rptr = next(m_rptr);
        //portEXIT_CRITICAL_ISR(&m_lock);
    }

    bool INTR_ATTR try_pop(value_type & v) {
        //portENTER_CRITICAL_ISR(&m_lock);
        index_type rptr = m_rptr;
        if (m_wptr == rptr) {
            //portEXIT_CRITICAL_ISR(&m_lock);
            return false;
        }
        v = m_buffer[rptr];
        m_rptr = next(rptr);
        //portEXIT_CRITICAL_ISR(&m_lock);
        return true;
    }

private:
    //portMUX_TYPE m_lock;
    volatile index_type m_rptr;
    volatile index_type m_wptr;
    volatile value_type m_buffer[capacity];

    static index_type INTR_ATTR prev(index_type v) { return index_type(v - 1) % Capacity; }

    static index_type INTR_ATTR next(index_type v) { return index_type(v + 1) % Capacity; }

    static index_type INTR_ATTR next(index_type v, index_type len) { return index_type(v + len) % Capacity; }

    static index_type INTR_ATTR dist(index_type ptr, index_type base) { return index_type(ptr - base) % Capacity; }
};
