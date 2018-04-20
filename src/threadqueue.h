#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

namespace mcp
{
namespace thread
{

    template <typename T>
    class ThreadQueue
    {
    public:

        ~ThreadQueue() {
            invalidate();
        }

        bool tryPop(T& out);
        bool waitPop(T& out);

        void push(T element);

        bool empty() const;
        bool isValid() const;

        void clear();
        void invalidate();

    private:
        std::atomic_bool mValid { true };

        mutable std::mutex mMutex;
        std::condition_variable mCondition;

        std::queue<T> mQueue;
    };

    template <typename T>
    bool ThreadQueue<T>::tryPop(T& out) {
        std::lock_guard<std::mutex> lock{mMutex};

        if (mQueue.empty() || !mValid) {
            return false;
        }

        out = std::move(mQueue.front());
        mQueue.pop();

        return true;
    }

    template <typename T>
    bool ThreadQueue<T>::waitPop(T& out) {
        std::unique_lock<std::mutex> lock{mMutex};

        mCondition.wait(lock, [this]() {
            return !mQueue.empty() || !mValid;
        });

        if (!mValid) {
            return false;
        }

        out = std::move(mQueue.front());
        mQueue.pop();

        return true;
    }

    template <typename T>
    void ThreadQueue<T>::push(T element) {
        std::lock_guard<std::mutex> lock{mMutex};

        mQueue.push(std::move(element));
        mCondition.notify_one();
    }

    template <typename T>
    bool ThreadQueue<T>::empty() const {
        std::lock_guard<std::mutex> lock{mMutex};

        return mQueue.empty();
    }

    template <typename T>
    bool ThreadQueue<T>::isValid() const {
        std::lock_guard<std::mutex> lock{mMutex};

        return mValid;
    }

    template <typename T>
    void ThreadQueue<T>::clear() {
        std::lock_guard<std::mutex> lock{mMutex};

        while (!mQueue.empty()) {
            mQueue.pop();
        }

        mCondition.notify_all();
    }

    template <typename T>
    void ThreadQueue<T>::invalidate() {
        std::lock_guard<std::mutex> lock{mMutex};

        mValid = false;
        mCondition.notify_all();
    }
}
}
