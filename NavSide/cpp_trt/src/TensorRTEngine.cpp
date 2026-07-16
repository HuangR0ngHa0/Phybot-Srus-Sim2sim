#include "TensorRTEngine.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace navside::trt {
using nvinfer1::DataType;

namespace {

std::string dtype_name(DataType dtype) {
    switch (dtype) {
        case DataType::kFLOAT:
            return "float32";
        case DataType::kHALF:
            return "float16";
        case DataType::kINT8:
            return "int8";
        case DataType::kINT32:
            return "int32";
        case DataType::kBOOL:
            return "bool";
        default:
            return "unknown";
    }
}

bool is_c_contiguous(PyArrayObject* array) {
    if (PyArray_NDIM(array) == 0) {
        return true;
    }
    ssize_t expected_stride = static_cast<ssize_t>(PyArray_ITEMSIZE(array));
    const ssize_t ndim = PyArray_NDIM(array);
    const npy_intp* dims = PyArray_DIMS(array);
    const npy_intp* strides = PyArray_STRIDES(array);
    for (ssize_t i = ndim - 1; i >= 0; --i) {
        if (strides[i] != expected_stride) {
            return false;
        }
        expected_stride *= static_cast<ssize_t>(dims[i]);
        if (i == 0) {
            break;
        }
    }
    return true;
}

void* checked_cuda_malloc(std::size_t bytes) {
    void* ptr = nullptr;
    const auto err = cudaMalloc(&ptr, bytes);
    if (err != cudaSuccess) {
        std::ostringstream oss;
        oss << "cudaMalloc failed: " << cudaGetErrorString(err);
        throw std::runtime_error(oss.str());
    }
    return ptr;
}

void* checked_cuda_malloc_host(std::size_t bytes) {
    void* ptr = nullptr;
    const auto err = cudaMallocHost(&ptr, bytes);
    if (err != cudaSuccess) {
        std::ostringstream oss;
        oss << "cudaMallocHost failed: " << cudaGetErrorString(err);
        throw std::runtime_error(oss.str());
    }
    return ptr;
}

void checked_cuda_free(void* ptr) {
    if (ptr != nullptr) {
        cudaFree(ptr);
    }
}

void checked_cuda_free_host(void* ptr) {
    if (ptr != nullptr) {
        cudaFreeHost(ptr);
    }
}

}  // namespace

void TensorRTEngine::Logger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        std::cerr << "[TensorRT] " << msg << std::endl;
    }
}

TensorRTEngine::TensorRTEngine(
    std::string engine_path,
    std::vector<TensorSpec> input_specs,
    std::vector<TensorSpec> output_specs
)
    : engine_path_(std::move(engine_path)),
      input_specs_(std::move(input_specs)),
      output_specs_(std::move(output_specs)) {
    load_engine();
    validate_contract();
    allocate_buffers();
}

TensorRTEngine::~TensorRTEngine() {
    free_buffers();
    if (context_ != nullptr) {
        delete context_;
        context_ = nullptr;
    }
    if (engine_ != nullptr) {
        delete engine_;
        engine_ = nullptr;
    }
    if (runtime_ != nullptr) {
        delete runtime_;
        runtime_ = nullptr;
    }
    if (stream_ != nullptr) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

TensorRTEngine::TensorRTEngine(TensorRTEngine&& other) noexcept
    : engine_path_(std::move(other.engine_path_)),
      input_specs_(std::move(other.input_specs_)),
      output_specs_(std::move(other.output_specs_)),
      buffers_(std::move(other.buffers_)),
      dtypes_(std::move(other.dtypes_)),
      logger_(),
      runtime_(other.runtime_),
      engine_(other.engine_),
      context_(other.context_),
      stream_(other.stream_) {
    other.runtime_ = nullptr;
    other.engine_ = nullptr;
    other.context_ = nullptr;
    other.stream_ = nullptr;
}

TensorRTEngine& TensorRTEngine::operator=(TensorRTEngine&& other) noexcept {
    if (this == &other) {
        return *this;
    }
    free_buffers();
    if (context_ != nullptr) {
        delete context_;
    }
    if (engine_ != nullptr) {
        delete engine_;
    }
    if (runtime_ != nullptr) {
        delete runtime_;
    }
    if (stream_ != nullptr) {
        cudaStreamDestroy(stream_);
    }

    engine_path_ = std::move(other.engine_path_);
    input_specs_ = std::move(other.input_specs_);
    output_specs_ = std::move(other.output_specs_);
    buffers_ = std::move(other.buffers_);
    dtypes_ = std::move(other.dtypes_);
    runtime_ = other.runtime_;
    engine_ = other.engine_;
    context_ = other.context_;
    stream_ = other.stream_;

    other.runtime_ = nullptr;
    other.engine_ = nullptr;
    other.context_ = nullptr;
    other.stream_ = nullptr;
    return *this;
}

std::size_t TensorRTEngine::element_count(const std::vector<ssize_t>& shape) {
    return std::accumulate(
        shape.begin(), shape.end(), static_cast<std::size_t>(1),
        [](std::size_t acc, ssize_t dim) {
            if (dim <= 0) {
                throw std::runtime_error("dynamic or invalid tensor dimension is not supported");
            }
            return acc * static_cast<std::size_t>(dim);
        }
    );
}

std::string TensorRTEngine::shape_to_string(const std::vector<ssize_t>& shape) {
    std::ostringstream oss;
    oss << "(";
    for (std::size_t i = 0; i < shape.size(); ++i) {
        oss << shape[i];
        if (i + 1 < shape.size()) {
            oss << ", ";
        }
    }
    oss << ")";
    return oss.str();
}

void TensorRTEngine::validate_array_contract(PyArrayObject* array, const TensorSpec& spec) {
    Py_buffer view;
    if (PyObject_GetBuffer(reinterpret_cast<PyObject*>(array), &view, PyBUF_FORMAT | PyBUF_ND | PyBUF_STRIDES) < 0) {
        throw std::runtime_error("Tensor " + spec.name + " must support the buffer protocol");
    }
    if (view.itemsize != static_cast<Py_ssize_t>(sizeof(float))) {
        PyBuffer_Release(&view);
        throw std::runtime_error("Tensor " + spec.name + " must be float32");
    }
    if (view.format == nullptr || (std::strcmp(view.format, "f") != 0 && std::strcmp(view.format, "=f") != 0 && std::strcmp(view.format, "<f") != 0)) {
        PyBuffer_Release(&view);
        throw std::runtime_error("Tensor " + spec.name + " must be float32");
    }
    if (view.ndim != static_cast<int>(spec.shape.size())) {
        std::ostringstream oss;
        oss << "Tensor " << spec.name << " must have rank " << spec.shape.size()
            << ", got " << view.ndim;
        PyBuffer_Release(&view);
        throw std::runtime_error(oss.str());
    }
    const npy_intp* dims = reinterpret_cast<const npy_intp*>(view.shape);
    for (ssize_t i = 0; i < view.ndim; ++i) {
        if (dims[i] != static_cast<npy_intp>(spec.shape[static_cast<std::size_t>(i)])) {
            std::ostringstream oss;
            oss << "Tensor " << spec.name << " must have exact shape "
                << shape_to_string(spec.shape) << ", got shape ";
            oss << "(";
            for (ssize_t j = 0; j < view.ndim; ++j) {
                oss << dims[j];
                if (j + 1 < view.ndim) {
                    oss << ", ";
                }
            }
            oss << ")";
            PyBuffer_Release(&view);
            throw std::runtime_error(oss.str());
        }
    }
    if (!PyBuffer_IsContiguous(&view, 'C')) {
        PyBuffer_Release(&view);
        throw std::runtime_error("Tensor " + spec.name + " must be C-contiguous");
    }
    PyBuffer_Release(&view);
}

nvinfer1::Dims TensorRTEngine::to_dims(const std::vector<ssize_t>& shape) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(shape.size());
    for (int i = 0; i < dims.nbDims; ++i) {
        dims.d[i] = static_cast<int>(shape[static_cast<std::size_t>(i)]);
    }
    return dims;
}

std::vector<ssize_t> TensorRTEngine::from_dims(const nvinfer1::Dims& dims) {
    std::vector<ssize_t> shape;
    shape.reserve(static_cast<std::size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        shape.push_back(static_cast<ssize_t>(dims.d[i]));
    }
    return shape;
}

std::size_t TensorRTEngine::dtype_size(DataType dtype) {
    if (dtype != DataType::kFLOAT) {
        std::ostringstream oss;
        oss << "Only float32 tensors are supported in this phase, got " << dtype_name(dtype);
        throw std::runtime_error(oss.str());
    }
    return sizeof(float);
}

void TensorRTEngine::load_engine() {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open TensorRT engine file: " + engine_path_);
    }
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (blob.empty()) {
        throw std::runtime_error("TensorRT engine file is empty: " + engine_path_);
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (runtime_ == nullptr) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }

    engine_ = runtime_->deserializeCudaEngine(blob.data(), blob.size());
    if (engine_ == nullptr) {
        throw std::runtime_error("Failed to deserialize TensorRT engine: " + engine_path_);
    }

    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
        throw std::runtime_error("Failed to create TensorRT execution context");
    }

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        throw std::runtime_error("Failed to create CUDA stream");
    }

    for (const auto& spec : input_specs_) {
        const auto dtype = engine_->getTensorDataType(spec.name.c_str());
        const auto shape = from_dims(engine_->getTensorShape(spec.name.c_str()));
        dtypes_[spec.name] = dtype;
        if (dtype != DataType::kFLOAT) {
            throw std::runtime_error("Input tensor " + spec.name + " must be float32");
        }
        if (shape != spec.shape) {
            throw std::runtime_error(
                "Input tensor " + spec.name + " shape mismatch: expected " +
                shape_to_string(spec.shape) + ", got " + shape_to_string(shape)
            );
        }
    }

    for (const auto& spec : output_specs_) {
        const auto dtype = engine_->getTensorDataType(spec.name.c_str());
        const auto shape = from_dims(engine_->getTensorShape(spec.name.c_str()));
        dtypes_[spec.name] = dtype;
        if (dtype != DataType::kFLOAT) {
            throw std::runtime_error("Output tensor " + spec.name + " must be float32");
        }
        if (shape != spec.shape) {
            throw std::runtime_error(
                "Output tensor " + spec.name + " shape mismatch: expected " +
                shape_to_string(spec.shape) + ", got " + shape_to_string(shape)
            );
        }
    }
}

void TensorRTEngine::validate_contract() {
    for (const auto& spec : input_specs_) {
        if (engine_->getTensorDataType(spec.name.c_str()) != DataType::kFLOAT) {
            throw std::runtime_error("Tensor " + spec.name + " must be float32");
        }
    }
    for (const auto& spec : output_specs_) {
        if (engine_->getTensorDataType(spec.name.c_str()) != DataType::kFLOAT) {
            throw std::runtime_error("Tensor " + spec.name + " must be float32");
        }
    }
}

void TensorRTEngine::allocate_buffers() {
    buffers_.clear();
    const auto allocate = [this](const TensorSpec& spec) {
        TensorBuffer buffer;
        buffer.shape = spec.shape;
        buffer.bytes = element_count(spec.shape) * dtype_size(DataType::kFLOAT);
        buffer.host = checked_cuda_malloc_host(buffer.bytes);
        buffer.device = checked_cuda_malloc(buffer.bytes);
        std::memset(buffer.host, 0, buffer.bytes);
        buffers_.emplace(spec.name, buffer);
    };

    for (const auto& spec : input_specs_) {
        allocate(spec);
    }
    for (const auto& spec : output_specs_) {
        allocate(spec);
    }
}

void TensorRTEngine::free_buffers() {
    for (auto& [_, buffer] : buffers_) {
        checked_cuda_free_host(buffer.host);
        checked_cuda_free(buffer.device);
        buffer.host = nullptr;
        buffer.device = nullptr;
        buffer.bytes = 0;
    }
    buffers_.clear();
}

std::vector<PyObject*> TensorRTEngine::infer(const std::vector<PyArrayObject*>& inputs) {
    if (inputs.size() != input_specs_.size()) {
        std::ostringstream oss;
        oss << "Expected " << input_specs_.size() << " inputs, got " << inputs.size();
        throw std::runtime_error(oss.str());
    }

    for (std::size_t i = 0; i < inputs.size(); ++i) {
        validate_array_contract(inputs[i], input_specs_[i]);
    }

    for (std::size_t i = 0; i < inputs.size(); ++i) {
        const auto& spec = input_specs_[i];
        const auto& buffer = buffers_.at(spec.name);
        std::memcpy(buffer.host, PyArray_DATA(inputs[i]), buffer.bytes);
        const auto copy_err = cudaMemcpyAsync(
            buffer.device,
            buffer.host,
            buffer.bytes,
            cudaMemcpyHostToDevice,
            stream_
        );
        if (copy_err != cudaSuccess) {
            std::ostringstream oss;
            oss << "cudaMemcpyAsync H2D failed for " << spec.name
                << ": " << cudaGetErrorString(copy_err);
            throw std::runtime_error(oss.str());
        }
        if (!context_->setTensorAddress(spec.name.c_str(), buffer.device)) {
            throw std::runtime_error("Failed to set input tensor address for " + spec.name);
        }
    }

    for (const auto& spec : output_specs_) {
        const auto& buffer = buffers_.at(spec.name);
        if (!context_->setTensorAddress(spec.name.c_str(), buffer.device)) {
            throw std::runtime_error("Failed to set output tensor address for " + spec.name);
        }
    }

    if (!context_->enqueueV3(stream_)) {
        throw std::runtime_error("Failed to enqueue TensorRT inference");
    }

    for (const auto& spec : output_specs_) {
        const auto& buffer = buffers_.at(spec.name);
        const auto copy_err = cudaMemcpyAsync(
            buffer.host,
            buffer.device,
            buffer.bytes,
            cudaMemcpyDeviceToHost,
            stream_
        );
        if (copy_err != cudaSuccess) {
            std::ostringstream oss;
            oss << "cudaMemcpyAsync D2H failed for " << spec.name
                << ": " << cudaGetErrorString(copy_err);
            throw std::runtime_error(oss.str());
        }
    }

    const auto sync_err = cudaStreamSynchronize(stream_);
    if (sync_err != cudaSuccess) {
        std::ostringstream oss;
        oss << "cudaStreamSynchronize failed: " << cudaGetErrorString(sync_err);
        throw std::runtime_error(oss.str());
    }

    std::vector<PyObject*> outputs;
    outputs.reserve(output_specs_.size());
    PyObject* numpy_module = PyImport_ImportModule("numpy");
    if (numpy_module == nullptr) {
        throw std::runtime_error("Failed to import numpy for output allocation");
    }
    PyObject* empty_func = PyObject_GetAttrString(numpy_module, "empty");
    PyObject* float32_obj = PyObject_GetAttrString(numpy_module, "float32");
    if (empty_func == nullptr || float32_obj == nullptr) {
        Py_XDECREF(empty_func);
        Py_XDECREF(float32_obj);
        Py_DECREF(numpy_module);
        throw std::runtime_error("Failed to resolve numpy.empty or numpy.float32");
    }
    try {
        for (const auto& spec : output_specs_) {
            const auto& buffer = buffers_.at(spec.name);
            PyObject* shape_tuple = PyTuple_New(static_cast<Py_ssize_t>(spec.shape.size()));
            if (shape_tuple == nullptr) {
                throw std::runtime_error("Failed to build output shape tuple for " + spec.name);
            }
            for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(spec.shape.size()); ++i) {
                PyObject* dim = PyLong_FromSsize_t(spec.shape[static_cast<std::size_t>(i)]);
                if (dim == nullptr) {
                    Py_DECREF(shape_tuple);
                    throw std::runtime_error("Failed to build output shape tuple for " + spec.name);
                }
                PyTuple_SET_ITEM(shape_tuple, i, dim);
            }
            PyObject* args = PyTuple_Pack(1, shape_tuple);
            PyObject* kwargs = Py_BuildValue("{sO}", "dtype", float32_obj);
            Py_DECREF(shape_tuple);
            if (args == nullptr || kwargs == nullptr) {
                Py_XDECREF(args);
                Py_XDECREF(kwargs);
                throw std::runtime_error("Failed to prepare numpy.empty call for " + spec.name);
            }
            PyObject* out = PyObject_Call(empty_func, args, kwargs);
            Py_DECREF(args);
            Py_DECREF(kwargs);
            if (out == nullptr) {
                throw std::runtime_error("Failed to allocate NumPy output array for " + spec.name);
            }
            Py_buffer out_view;
            if (PyObject_GetBuffer(out, &out_view, PyBUF_WRITABLE | PyBUF_ND | PyBUF_STRIDES) < 0) {
                Py_DECREF(out);
                throw std::runtime_error("Failed to access NumPy output buffer for " + spec.name);
            }
            std::memcpy(out_view.buf, buffer.host, buffer.bytes);
            PyBuffer_Release(&out_view);
            outputs.push_back(out);
        }
    } catch (...) {
        for (PyObject* obj : outputs) {
            Py_XDECREF(obj);
        }
        Py_DECREF(empty_func);
        Py_DECREF(float32_obj);
        Py_DECREF(numpy_module);
        throw;
    }
    Py_DECREF(empty_func);
    Py_DECREF(float32_obj);
    Py_DECREF(numpy_module);
    return outputs;
}

}  // namespace navside::trt
