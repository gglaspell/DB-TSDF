#ifndef DB_TSDF_EXPORT_WORKER_HPP
#define DB_TSDF_EXPORT_WORKER_HPP

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace db_tsdf {
struct ExportStatus {
    uint64_t id;
    std::string path, state, error;
};

class ExportWorker {
public:
    explicit ExportWorker(size_t capacity=2) : capacity_(capacity), thread_([this]{run();}) {}
    ~ExportWorker() { shutdown(); }
    ExportWorker(const ExportWorker&) = delete;
    ExportWorker& operator=(const ExportWorker&) = delete;
    uint64_t enqueue(std::string path,std::function<void()> action) {
        std::lock_guard lock(mutex_);
        if (stopping_ || queue_.size()+running_ >= capacity_) return 0;
        const auto id=++next_;
        queue_.push_back({id,std::move(action)});
        statuses_.push_back({id,std::move(path),"queued",""});
        while (statuses_.size()>16) statuses_.pop_front();
        changed_.notify_one();
        return id;
    }
    std::vector<ExportStatus> statuses() const {
        std::lock_guard lock(mutex_);
        return {statuses_.begin(),statuses_.end()};
    }
    void shutdown() {
        { std::lock_guard lock(mutex_); stopping_=true; }
        changed_.notify_one();
        if (thread_.joinable()) thread_.join();
    }
private:
    struct Job { uint64_t id; std::function<void()> action; };
    void status(uint64_t id,const std::string& state,const std::string& error={}) {
        for (auto& s : statuses_) if (s.id==id) { s.state=state; s.error=error; break; }
    }
    void run() {
        std::unique_lock lock(mutex_);
        for (;;) {
            changed_.wait(lock,[&]{return stopping_ || !queue_.empty();});
            if (queue_.empty()) return;
            auto job=std::move(queue_.front()); queue_.pop_front(); running_=true;
            status(job.id,"running");
            lock.unlock();
            std::string error;
            try { job.action(); }
            catch (const std::exception& e) { error=e.what(); }
            catch (...) { error="unknown export failure"; }
            lock.lock();
            status(job.id,error.empty()?"complete":"failed",error);
            running_=false;
        }
    }
    size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Job> queue_;
    std::deque<ExportStatus> statuses_;
    bool running_{false},stopping_{false};
    uint64_t next_{0};
    std::thread thread_;
};
}
#endif
