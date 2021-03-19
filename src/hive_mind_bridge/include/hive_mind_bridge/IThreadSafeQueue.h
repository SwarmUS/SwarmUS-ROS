#ifndef HIVE_MIND_BRIDGE_ITHREADSAFEQUEUE_H
#define HIVE_MIND_BRIDGE_ITHREADSAFEQUEUE_H

template <class T>
class IThreadSafeQueue {
public:
    virtual void push(const T& item) = 0;

    virtual void pop() = 0;

    virtual T front() = 0;

    virtual T back() = 0;

    virtual size_t size() = 0;

    virtual bool empty() = 0;
};

#endif //HIVE_MIND_BRIDGE_ITHREADSAFEQUEUE_H
