//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_DORA_NODE_H
#define LIVOX_DORA_NODE_H

#include <memory>
#include <mutex>
#include <string>

namespace livox_ros {

class LddcDora;

class DoraNode final {
 public:
  explicit DoraNode(void* dora_context);
  DoraNode(const DoraNode &) = delete;
  ~DoraNode();
  DoraNode &operator=(const DoraNode &) = delete;

  /**
   * @brief Send data through Dora.
   *
   * Thread-safe: may be called concurrently from PcdPollThread and
   * ImuPollThread. An internal mutex serialises all calls to
   * dora_send_output() because the underlying Dora C API (and its TCP
   * socket) is NOT thread-safe.
   */
  int SendOutput(const std::string& topic, const std::string& data);

  // Get Dora context
  void* GetDoraContext() const { return dora_context_; }

 public:
  std::unique_ptr<LddcDora> lddc_ptr_;

 private:
  void*      dora_context_;
  std::mutex send_mutex_;   // serialises all dora_send_output() calls
};

} // namespace livox_ros

#endif // LIVOX_DORA_NODE_H