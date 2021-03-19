#ifndef HIVE_MIND_BRIDGE_THREADSAFEQUEUE_H
#define HIVE_MIND_BRIDGE_THREADSAFEQUEUE_H

#include <mutex>
#include <queue>

template <class T>
class ThreadSafeQueue {
  public:
    ThreadSafeQueue() {}

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(item);
    }

    void pop() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.pop();
    }

    T front() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.front();
    }

    T back() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.back();
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

  private:
    std::mutex m_mutex;
    std::queue<T> m_queue;
};

#endif // HIVE_MIND_BRIDGE_THREADSAFEQUEUE_H
