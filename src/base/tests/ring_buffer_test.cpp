#include "ringbuffer/ring_buffer.hpp"

#include <cassert>
#include <iostream>

int main()
{
    robot_base::RingBuffer<int> buffer(3);

    assert(buffer.empty());
    assert(buffer.size() == 0);
    assert(buffer.capacity() == 3);

    assert(buffer.push_back(1));
    assert(buffer.push_back(2));
    assert(buffer.push_back(3));
    assert(buffer.full());
    assert(!buffer.push_back(4));

    assert(buffer.front() == 1);
    buffer.pop_front();
    assert(buffer.front() == 2);
    buffer.pop_front();

    assert(buffer.push_back(4));
    assert(buffer.push_back(5));
    assert(buffer.full());

    assert(buffer.front() == 3);
    buffer.pop_front();
    assert(buffer.front() == 4);
    buffer.pop_front();
    assert(buffer.front() == 5);
    buffer.pop_front();

    assert(buffer.empty());
    assert(buffer.size() == 0);

    std::cout << "ring_buffer_test passed\n";
    return 0;
}
