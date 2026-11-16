#pragma once
#include <chrono>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace ros2_linkattacher {
// Service threads submit work; only Gazebo's world-update thread executes it.
// A timed-out pending operation is cancelled before it can change physics.
class WorldOperations {
  struct Job {
    std::mutex mutex;
    bool cancelled{false}, done{false};
    std::function<void()> action;
    std::promise<void> completed;
    std::exception_ptr error;
  };
  std::mutex mutex_;
  std::deque<std::shared_ptr<Job>> pending_;
public:
  template<class T,class F>
  T invoke(F action,std::chrono::milliseconds timeout) {
    auto result=std::make_shared<T>();auto job=std::make_shared<Job>();
    job->action=[result,action](){*result=action();};
    auto future=job->completed.get_future();
    {std::lock_guard<std::mutex> lock(mutex_);pending_.push_back(job);}
    future.wait_for(timeout);
    std::lock_guard<std::mutex> lock(job->mutex);
    if(!job->done) {
      job->cancelled=true;
      throw std::runtime_error("Gazebo world update unavailable; operation cancelled");
    }
    if(job->error)std::rethrow_exception(job->error);
    return *result;
  }
  void drain() {
    std::deque<std::shared_ptr<Job>> jobs;
    {std::lock_guard<std::mutex> lock(mutex_);jobs.swap(pending_);}
    for(auto & job:jobs) {
      std::lock_guard<std::mutex> lock(job->mutex);
      if(!job->cancelled) {
        try{job->action();}catch(...){job->error=std::current_exception();}
      }
      job->done=true;job->completed.set_value();
    }
  }
};
}
