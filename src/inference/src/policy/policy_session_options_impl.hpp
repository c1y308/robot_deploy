#pragma once

#include "policy/policy_execution_setup.hpp"

#include <onnxruntime_cxx_api.h>

namespace inference {

struct PolicySessionOptions::Impl {
    Ort::SessionOptions options;
};

}  // namespace inference
