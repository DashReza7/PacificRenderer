#include "core/Sampler.h"

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::vector<Sampler> rngs;  // One RNG per thread
    std::queue<std::function<void(Sampler&)>> tasks;

    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;

public:
    // Constructor: creates thread pool with specified number of threads
    // If num_threads is 0, uses hardware concurrency
    // master_samapler is used for seeding each rng
    ThreadPool(Sampler& master_sampler, size_t num_threads = 0) : stop(false) {
        if (num_threads == 0)
            num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0)
            num_threads = 1;

        workers.reserve(num_threads);
        rngs.reserve(num_threads);

        for (size_t i = 0; i < num_threads; ++i) {
            // Create a unique RNG for this thread
            Float sample_val = master_sampler.get_1D();  // [0,1)
            // convert to uint64_t seed
            // FIXME: this gives poor randomness
            uint64_t seed = static_cast<uint64_t>(sample_val * 1e6);
            rngs.emplace_back(seed, master_sampler.spp);

            workers.emplace_back([this, i] {
                // This thread's dedicated RNG (no sharing!)
                Sampler& rng = rngs[i];

                while (true) {
                    std::function<void(Sampler&)> task;

                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);

                        // Wait until there's a task or we're stopping
                        condition.wait(lock, [this] {
                            return stop || !tasks.empty();
                        });

                        if (stop && tasks.empty()) {
                            return;
                        }

                        task = std::move(tasks.front());
                        tasks.pop();
                    }

                    // Execute task with this thread's RNG
                    task(rng);
                }
            });
        }
    }

    // Add a job to the queue
    // Your function should take Sampler& as its first parameter
    // Returns a future that will contain the result
    template <class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result<F, Sampler&, Args...>::type> {
        using return_type = typename std::invoke_result<F, Sampler&, Args...>::type;

        auto task = std::make_shared<std::packaged_task<return_type(Sampler&)>>(
            std::bind(std::forward<F>(f), std::placeholders::_1, std::forward<Args>(args)...));

        std::future<return_type> res = task->get_future();

        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            if (stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }

            tasks.emplace([task](Sampler& rng) { (*task)(rng); });
        }

        condition.notify_one();
        return res;
    }

    // Destructor: waits for all threads to finish
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }

        condition.notify_all();

        for (std::thread& worker : workers) {
            worker.join();
        }
    }
};
