#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "threadqueue.h"

namespace mcp
{
namespace thread
{

    class ThreadPool
    {
    public:
        template <typename T>
        class TaskFuture
        {
        public:
            TaskFuture(std::future<T>&& future)
                : mFuture{std::move(future)}
            {
            }

            ~TaskFuture() {
                if (mFuture.valid()) {
                    mFuture.get();
                }
            }

            TaskFuture(const TaskFuture& rhs) = delete;
            TaskFuture(TaskFuture&& other)    = default;

            TaskFuture& operator= (const TaskFuture& rhs) = delete;
            TaskFuture& operator= (TaskFuture&& other)    = default;

            auto get() {
                return mFuture.get();
            }

        private:
            std::future<T> mFuture;
        };

    private:
        class IThreadTask
        {
        public:
            IThreadTask()          = default;
            virtual ~IThreadTask() = default;

            IThreadTask(const IThreadTask& rhs) = delete;
            IThreadTask(IThreadTask&& other)    = default;

            IThreadTask& operator= (const IThreadTask& rhs) = delete;
            IThreadTask& operator= (IThreadTask&& other)    = default;

            virtual void run() = 0;
        };

        template <typename Func>
        class ThreadTask : public IThreadTask
        {
        public:
            ThreadTask(Func&& func)
                : mFunc{std::move(func)}
            {
            }

            ~ThreadTask() override = default;

            ThreadTask(const ThreadTask& rhs) = delete;
            ThreadTask(ThreadTask&& other)    = default;

            ThreadTask& operator= (const ThreadTask& rhs) = delete;
            ThreadTask& operator= (ThreadTask&& rhs)      = default;

            void run() override {
                mFunc();
            }

        private:
            Func mFunc;
        };

    public:
        ThreadPool();
        explicit ThreadPool(const uint32_t numThreads);
        ThreadPool(const ThreadPool& rhs) = delete;

        ~ThreadPool() {
            destroy();
        }

        ThreadPool& operator= (const ThreadPool& rhs) = delete;

        template <typename Func, typename... Args>
        auto submit(Func&& func, Args&&... args);

    private:
        void worker();
        void destroy();

        std::atomic_bool mDone;

        ThreadQueue<std::unique_ptr<IThreadTask>> mQueue;
        std::vector<std::thread> mThreads;
    };

    ThreadPool::ThreadPool()
        : ThreadPool {std::max(std::thread::hardware_concurrency(), 2u) - 1u}
    {
    }

    ThreadPool::ThreadPool(const uint32_t numThreads)
        : mDone{false}
        , mQueue{}
        , mThreads{}
    {
        try {
            for (uint32_t i = 0u; i < numThreads; ++i) {
                mThreads.emplace_back(&ThreadPool::worker, this);
            }
        } catch(...) {
            destroy();
            throw;
        }

        std::cout << "Spawned ThreadPool with " << numThreads << " threads" << std::endl;
    }

    template <typename Func, typename... Args>
    auto ThreadPool::submit(Func&& func, Args&&... args) {
        auto boundTask = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);

        using ResultType   = std::result_of_t<decltype(boundTask)()>;
        using PackagedTask = std::packaged_task<ResultType()>;
        using TaskType     = ThreadTask<PackagedTask>;

        PackagedTask task{std::move(boundTask)};
        TaskFuture<ResultType> result{task.get_future()};

        mQueue.push(std::make_unique<TaskType>(std::move(task)));

        return result;
    }

    void ThreadPool::worker() {
        while (!mDone) {
            std::unique_ptr<IThreadTask> pTask{nullptr};

            if (mQueue.waitPop(pTask)) {
                pTask->run();
            }
        }
    }

    void ThreadPool::destroy() {
        mDone = true;
        mQueue.invalidate();

        for (auto& thread : mThreads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

}
}
