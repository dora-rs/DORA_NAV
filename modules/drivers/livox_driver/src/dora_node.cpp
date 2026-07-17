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

#include "dora_node.h"
#include "lddc_dora.h"
#include <iostream>

extern "C"{
#include "node_api.h"
}

namespace livox_ros {

DoraNode::DoraNode(void* dora_context) : dora_context_(dora_context) {
  std::cout << "DoraNode initialized" << std::endl;
}

DoraNode::~DoraNode() {
  std::cout << "DoraNode destroyed" << std::endl;
}

int DoraNode::SendOutput(const std::string& topic, const std::string& data) {
  if (dora_context_ == nullptr) {
    std::cerr << "Dora context is null" << std::endl;
    return -1;
  }

  // Dora's C API writes to a single TCP socket — not thread-safe.
  // Lock here so PcdPollThread and ImuPollThread never interleave their writes.
  std::lock_guard<std::mutex> lock(send_mutex_);

  int result = dora_send_output(dora_context_,
                               const_cast<char*>(topic.c_str()), topic.length(),
                               const_cast<char*>(data.data()), data.size());

  if (result != 0) {
    std::cerr << "Failed to send output to topic: " << topic << std::endl;
  }

  return result;
}

} // namespace livox_ros
