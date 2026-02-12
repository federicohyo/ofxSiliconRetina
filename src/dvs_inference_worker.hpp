#pragma once
/// @file dvs_inference_worker.hpp
/// @brief Thread-pool worker for asynchronous ONNX inference.
///
/// Decouples inference latency from the rendering frame rate.
/// The main thread builds input tensors and submits jobs; the worker thread
/// runs ONNX inference and writes results under a mutex.  The main thread
/// reads the last completed result for drawing (never blocks on ONNX).

#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <iostream>

namespace dvs {

/// Generic asynchronous inference worker.
/// ResultT must be default-constructible and move-assignable.
template <typename ResultT>
class InferenceWorker {
public:
    using JobFn = std::function<ResultT()>;

    InferenceWorker() = default;

    ~InferenceWorker() { stop(); }

    /// Start the worker thread (call once).
    void start() {
        if (running_) return;
        running_ = true;
        thread_ = std::thread(&InferenceWorker::loop_, this);
    }

    /// Submit a job.  If the worker is still busy with the previous job,
    /// the submission is silently dropped (skip frame rather than block).
    /// @return true if the job was accepted, false if dropped.
    bool submit(JobFn job) {
        if (!running_) return false;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (busy_) return false;  // previous job still running
            pending_ = std::move(job);
            has_pending_ = true;
        }
        cv_.notify_one();
        return true;
    }

    /// Check whether the worker is currently running a job.
    bool isBusy() const { return busy_.load(); }

    /// Read the last completed result (thread-safe).
    ResultT lastResult() const {
        std::lock_guard<std::mutex> lk(result_mu_);
        return result_;
    }

    /// Check if at least one result has been produced.
    bool hasResult() const { return has_result_.load(); }

    /// Stop the worker thread and join.
    void stop() {
        if (!running_) return;
        {
            std::lock_guard<std::mutex> lk(mu_);
            running_ = false;
        }
        cv_.notify_one();
        if (thread_.joinable()) thread_.join();
    }

private:
    void loop_() {
        while (true) {
            JobFn job;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&] { return has_pending_ || !running_; });
                if (!running_ && !has_pending_) break;
                job = std::move(pending_);
                has_pending_ = false;
                busy_ = true;
            }

            ResultT r;
            try {
                r = job();
            } catch (const std::exception& ex) {
                std::cerr << "[InferenceWorker] job exception: " << ex.what() << std::endl;
                busy_ = false;
                continue;
            } catch (...) {
                std::cerr << "[InferenceWorker] unknown job exception" << std::endl;
                busy_ = false;
                continue;
            }

            {
                std::lock_guard<std::mutex> lk(result_mu_);
                result_ = std::move(r);
                has_result_ = true;
            }
            busy_ = false;
        }
    }

    std::thread thread_;
    std::mutex mu_;
    mutable std::mutex result_mu_;
    std::condition_variable cv_;

    bool running_ = false;
    std::atomic<bool> busy_{false};
    std::atomic<bool> has_result_{false};
    bool has_pending_ = false;

    JobFn pending_;
    ResultT result_;
};

} // namespace dvs
