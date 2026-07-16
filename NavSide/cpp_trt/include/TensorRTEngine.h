#ifndef NAVSIDE_CPP_TRT_TENSORRTENGINE_H
#define NAVSIDE_CPP_TRT_TENSORRTENGINE_H

#include <Python.h>
#include <numpy/arrayobject.h>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace navside::trt {
struct TensorSpec {
    std::string name;
    std::vector<ssize_t> shape;
};

struct TensorBuffer {
    void* host{nullptr};
    void* device{nullptr};
    std::size_t bytes{0};
    std::vector<ssize_t> shape;
};

class TensorRTEngine {
public:
    TensorRTEngine(
        std::string engine_path,
        std::vector<TensorSpec> input_specs,
        std::vector<TensorSpec> output_specs
    );
    ~TensorRTEngine();

    TensorRTEngine(const TensorRTEngine&) = delete;
    TensorRTEngine& operator=(const TensorRTEngine&) = delete;
    TensorRTEngine(TensorRTEngine&&) noexcept;
    TensorRTEngine& operator=(TensorRTEngine&&) noexcept;

    const std::string& engine_path() const { return engine_path_; }
    const std::vector<TensorSpec>& input_specs() const { return input_specs_; }
    const std::vector<TensorSpec>& output_specs() const { return output_specs_; }

    std::vector<PyObject*> infer(const std::vector<PyArrayObject*>& inputs);

private:
    class Logger final : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override;
    };

    void load_engine();
    void validate_contract();
    void allocate_buffers();
    void free_buffers();

    static std::size_t element_count(const std::vector<ssize_t>& shape);
    static std::string shape_to_string(const std::vector<ssize_t>& shape);
    static void validate_array_contract(PyArrayObject* array, const TensorSpec& spec);
    static nvinfer1::Dims to_dims(const std::vector<ssize_t>& shape);
    static std::vector<ssize_t> from_dims(const nvinfer1::Dims& dims);
    static std::size_t dtype_size(nvinfer1::DataType dtype);

    std::string engine_path_;
    std::vector<TensorSpec> input_specs_;
    std::vector<TensorSpec> output_specs_;
    std::unordered_map<std::string, TensorBuffer> buffers_;
    std::unordered_map<std::string, nvinfer1::DataType> dtypes_;

    Logger logger_;
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    cudaStream_t stream_{nullptr};
};

}  // namespace navside::trt

#endif  // NAVSIDE_CPP_TRT_TENSORRTENGINE_H
