#ifndef NAVSIDE_CPP_TRT_NAVSIDETRTRUNNER_H
#define NAVSIDE_CPP_TRT_NAVSIDETRTRUNNER_H

#include "TensorRTEngine.h"

#include <string>

namespace navside::trt {

class NavSideTRTRunner {
public:
    NavSideTRTRunner(const std::string& encoder_engine_path, const std::string& policy_engine_path);

    PyObject* encode_depth(PyArrayObject* depth_tensor);
    PyObject* run_policy(PyArrayObject* obs, PyArrayObject* h_in, PyArrayObject* c_in);

private:
    TensorRTEngine encoder_engine_;
    TensorRTEngine policy_engine_;
};

}  // namespace navside::trt

#endif  // NAVSIDE_CPP_TRT_NAVSIDETRTRUNNER_H
