#pragma once

// std default allocator
// based on https://stackoverflow.com/a/52255429

template<class T>
class allocator {
public:
    typedef T value_type;
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    // rebind allocator to type U
    template <class U>
    struct rebind {
        typedef allocator<U> other;
    };

    // return address of values
    pointer address (reference value) const {
        return &value;
    }
    const_pointer address (const_reference value) const {
        return &value;
    }

    /* constructors and destructor
    * - nothing to do because the allocator has no state
    */
    allocator() throw() {
    }
    allocator(const allocator&) throw() {
    }
    template <class U>
    allocator(const allocator<U>&) throw() {
    }
    ~allocator() throw() {
    }

    // return maximum number of elements that can be allocated
    size_type max_size () const throw() {
        return 176*1024 / sizeof(T); // 176 kB is maximum allowed chunk in ESP32 RAM
    }

    // allocate but don't initialize num elements of type T
    pointer allocate (size_type num, const void* = 0) {
        return (pointer)(::operator new(num*sizeof(T)));
    }

    // initialize elements of allocated storage p with value value
    void construct (pointer p, const T& value) {
        // initialize memory with placement new
        new((void*)p)T(value);
    }

    // destroy elements of initialized storage p
    void destroy (pointer p) {
        // destroy objects by calling their destructor
        p->~T();
    }

    // deallocate storage p of deleted elements
    void deallocate (pointer p, size_type num) {
        ::operator delete((void*)p);
    }
};
/*
template<class T, size_t Capacity>
class StaticAllocator {
public:
    typedef T value_type;
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    // rebind allocator to type U
    template <class U>
    struct rebind {
        typedef StaticAllocator<U, Capacity> other;
    };

    // return address of values
    pointer address (reference value) const {
        return &value;
    }
    const_pointer address (const_reference value) const {
        return &value;
    }

    // constructors and destructor
    StaticAllocator() throw() {
        for (size_type i = 0; i < max_size(); i += 8)
            m_occupied[i] = 0;
    }
    ~StaticAllocator() throw() {
    }

    // return maximum number of elements that can be allocated
    size_type INTR_ATTR max_size () const throw() {
        return Capacity;
    }

    // allocate but don't initialize num elements of type T
    pointer INTR_ATTR allocate (const size_type num, const void* = 0) {
        ISRGuard guard;
        if (num == 1) {
            for (size_type i = 0; i < max_size() / 8; ++i) {
                if (m_occupied[i] == 0xFF)
                    continue;
                for (uint8_t j = 1, k = 0; 1; j <<= 1, ++k)
                    if ((m_occupied[i] & j) == 0)
                        return reinterpret_cast<pointer>(m_pool + (i * 8 + k) * sizeof(T));
            }
        } else {
            size_type available = 0;
            for (size_type i = 0; i != max_size(); ++i) {
                if (m_occupied[i/8] & (1<<(i%8)) == 0) {
                    if (++available == num) {
                        for (; available != 0; --available) {
                            const size_type j = i-available+1;
                            m_occupied[j/8] |= (1<<(j%8));
                        return reinterpret_cast<pointer>(m_pool + (i - num + 1) * sizeof(T));
                    }
                } else {
                    available = 0;
                }
            }
        }
        return nullptr;
    }

    // initialize elements of allocated storage p with value value
    void INTR_ATTR construct (pointer p, const T& value) {
        // initialize memory with placement new
        new((void*)p)T(value);
    }

    // destroy elements of initialized storage p
    void INTR_ATTR destroy (pointer p) {
        // destroy objects by calling their destructor
        p->~T();
    }

    // deallocate storage p of deleted elements
    void INTR_ATTR deallocate (pointer p, size_type num) {
        ISRGuard guard;
        size_type index = reinterpret_cast<size_type>(reinterpret_cast<uint8_t*>(p) - m_pool);
        for (size_type i = 0; i != num, ++i) {
            m_occupied[index/8] &= ~(1<<(index%8));
            ++index;
        }
    }
private:
    uint8_t m_pool[Capacity * sizeof(T)];
    volatile uint8_t m_occupied[(Capacity / 8) + ((Capacity % 8) == 0) ? 0 : 1];
};*/