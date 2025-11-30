/**
 * @brief Thread-safe queue wrapper.
 * 
 * Provides a standard queue interface (push, pop, front, empty, clear)
 * with internal locking to allow safe access from multiple threads.
 */

#ifndef SAFEQUEUE_H
#define SAFEQUEUE_H

#include <queue>
#include <mutex>
#include <optional>

template<typename T>
class SafeQueue {
private:
    std::queue<T> q;
    mutable std::mutex mtx; // mutable allows locking in const methods like empty()

public:
    SafeQueue() = default;
    ~SafeQueue() = default;

    // Disable copy
    SafeQueue(const SafeQueue&) = delete;
    SafeQueue& operator=(const SafeQueue&) = delete;

    // Push elements
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mtx);
        q.push(value);
    }

    void push(T&& value) {
        std::lock_guard<std::mutex> lock(mtx);
        q.push(std::move(value));
    }

    // Pop first element or return std::nullopt if empty
    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (q.empty()) return std::nullopt;
        T value = std::move(q.front());
        q.pop();
        return value;
    }
    
    std::optional<T> frontIfExists() {
        std::lock_guard<std::mutex> lock(mtx);
        if (q.empty()) return std::nullopt;
        return q.front();  // returns a copy
    }

    // Get queue size
    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return q.size();
    }

    // Optional: clear all elements
    void clear() {
        std::lock_guard<std::mutex> lock(mtx);
        std::queue<T> emptyQueue;
        std::swap(q, emptyQueue);
    }
};

#endif // SAFEQUEUE_H
