//
// Created by xiang on 2022/2/9.
//

#pragma once

#include <glog/logging.h>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "common/eigen_types.h"
#include "common/std_types.h"

namespace lightning {

/**
 * 异步消息处理类
 * 内部有线程和队列机制，保证回调是串行的
 * 主要用于分离各模块线程，避免写一堆锁或者条件变量
 *
 * @tparam T
 *
 * NOTE skip设为1的时候实际不会跳帧。。设为2的时候实际走一帧跳一帧
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;  // 消息回调函数
    AsyncMessageProcess() = default;
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");
    ~AsyncMessageProcess() { Quit(); }

    /// 设置处理函数
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// 设置队列最大长度
    void SetMaxSize(size_t size) { max_size_ = size; }

    /// 开始处理消息
    void Start();

    /// 添加一条消息
    void AddMessage(const T& msg);

    /// 退出
    void Quit();

    /// 清空跳帧计数器，下一个数据会立即执行
    void CleanSkipCnt();

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) { enable_skip_ = enable_skip, skip_num_ = skip_num; }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::deque<T> msg_buffer_;
    bool update_flag_ = false;
    bool exit_flag_ = false;
    size_t max_size_ = 40;
    std::string name_;

    /// 跳帧
    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(AsyncMessageProcess::ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    exit_flag_ = false;
    update_flag_ = false;
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    while (!exit_flag_) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return update_flag_; });

        // take the message and process it
        auto buffer = msg_buffer_;
        msg_buffer_.clear();
        update_flag_ = false;
        lock.unlock();

        // 处理之
        for (const auto& msg : buffer) {
            custom_func_(msg);
        }
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg) {
    UL lock(mutex_);
    if (enable_skip_) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_ % skip_num_;
            return;
        }

        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    msg_buffer_.push_back(msg);
    while (msg_buffer_.size() > max_size_) {
        LOG(ERROR) << name_ << " exceeds largest size: " << max_size_;
        msg_buffer_.pop_front();
    }

    update_flag_ = true;
    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    update_flag_ = true;
    exit_flag_ = true;
    cv_msg_.notify_one();

    if (proc_.joinable()) {
        proc_.join();
    }
}

}  // namespace lightning
