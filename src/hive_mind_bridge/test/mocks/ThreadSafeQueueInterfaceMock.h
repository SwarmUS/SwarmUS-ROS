#ifndef HIVE_MIND_BRIDGE_THREADSAFEQUEUEINTERFACEMOCK_H
#define HIVE_MIND_BRIDGE_THREADSAFEQUEUEINTERFACEMOCK_H

//#include <gmock/gmock.h>
#include "hive_mind_bridge/IThreadSafeQueue.h"

template <class T>
class ThreadSafeQueueInterfaceMock : public IThreadSafeQueue<T>{
public:
    ~ThreadSafeQueueInterfaceMock() = default;

    MOCK_METHOD(void, push, (const T& item), (override));

    MOCK_METHOD(void, pop, (), (override));

    MOCK_METHOD(T, front, (), (override));

    MOCK_METHOD(T, back, (), (override));

    MOCK_METHOD(size_t, size, (), (override));

    MOCK_METHOD(bool, empty, (), (override));
};

#endif //HIVE_MIND_BRIDGE_THREADSAFEQUEUEINTERFACEMOCK_H
