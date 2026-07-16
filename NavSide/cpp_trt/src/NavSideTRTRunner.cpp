#include "NavSideTRTRunner.h"

#include <Python.h>

#include <utility>

namespace navside::trt {

namespace {

TensorSpec encoder_input_spec() {
    return TensorSpec{"depth", {1, 1, 40, 64}};
}

TensorSpec encoder_output_spec() {
    return TensorSpec{"depth_features", {1, 64, 5, 8}};
}

TensorSpec policy_obs_spec() {
    return TensorSpec{"obs", {1, 2576}};
}

TensorSpec policy_h_in_spec() {
    return TensorSpec{"h_in", {1, 1, 512}};
}

TensorSpec policy_c_in_spec() {
    return TensorSpec{"c_in", {1, 1, 512}};
}

TensorSpec policy_actions_spec() {
    return TensorSpec{"actions", {1, 3}};
}

TensorSpec policy_h_out_spec() {
    return TensorSpec{"h_out", {1, 1, 512}};
}

TensorSpec policy_c_out_spec() {
    return TensorSpec{"c_out", {1, 1, 512}};
}

}  // namespace

NavSideTRTRunner::NavSideTRTRunner(const std::string& encoder_engine_path, const std::string& policy_engine_path)
    : encoder_engine_(
          encoder_engine_path,
          {encoder_input_spec()},
          {encoder_output_spec()}
      ),
      policy_engine_(
          policy_engine_path,
          {policy_obs_spec(), policy_h_in_spec(), policy_c_in_spec()},
          {policy_actions_spec(), policy_h_out_spec(), policy_c_out_spec()}
      ) {}

PyObject* NavSideTRTRunner::encode_depth(PyArrayObject* depth_tensor) {
    auto outputs = encoder_engine_.infer({depth_tensor});
    return outputs.at(0);
}

PyObject* NavSideTRTRunner::run_policy(PyArrayObject* obs, PyArrayObject* h_in, PyArrayObject* c_in) {
    auto outputs = policy_engine_.infer({obs, h_in, c_in});
    PyObject* tuple = PyTuple_New(static_cast<Py_ssize_t>(outputs.size()));
    if (tuple == nullptr) {
        for (PyObject* obj : outputs) {
            Py_XDECREF(obj);
        }
        return nullptr;
    }
    for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(outputs.size()); ++i) {
        PyTuple_SET_ITEM(tuple, i, outputs[static_cast<std::size_t>(i)]);
    }
    return tuple;
}

}  // namespace navside::trt
