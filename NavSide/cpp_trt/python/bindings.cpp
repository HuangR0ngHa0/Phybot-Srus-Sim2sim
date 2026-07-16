#include <Python.h>
#include <numpy/arrayobject.h>

#include "NavSideTRTRunner.h"

#include <new>

namespace {

using navside::trt::NavSideTRTRunner;

typedef struct {
    PyObject_HEAD
    NavSideTRTRunner* runner;
} PyNavSideTRTRunnerObject;

PyObject* raise_runtime_error(const std::exception& exc) {
    PyErr_SetString(PyExc_RuntimeError, exc.what());
    return nullptr;
}

PyArrayObject* require_array(PyObject* obj, const char* name) {
    if (!PyArray_Check(obj)) {
        PyErr_Format(PyExc_TypeError, "%s must be a NumPy ndarray", name);
        return nullptr;
    }
    return reinterpret_cast<PyArrayObject*>(obj);
}

PyNavSideTRTRunnerObject* require_runner(PyObject* self) {
    auto* obj = reinterpret_cast<PyNavSideTRTRunnerObject*>(self);
    if (obj->runner == nullptr) {
        PyErr_SetString(PyExc_RuntimeError, "NavSideTRTRunner is not initialized");
        return nullptr;
    }
    return obj;
}

PyObject* NavSideTRTRunner_new(PyTypeObject* type, PyObject*, PyObject*) {
    auto* self = reinterpret_cast<PyNavSideTRTRunnerObject*>(type->tp_alloc(type, 0));
    if (self != nullptr) {
        self->runner = nullptr;
    }
    return reinterpret_cast<PyObject*>(self);
}

int NavSideTRTRunner_init(PyNavSideTRTRunnerObject* self, PyObject* args, PyObject* kwargs) {
    static const char* kwlist[] = {"encoder_engine_path", "policy_engine_path", nullptr};
    const char* encoder_path = nullptr;
    const char* policy_path = nullptr;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "ss", const_cast<char**>(kwlist), &encoder_path, &policy_path)) {
        return -1;
    }
    try {
        delete self->runner;
        self->runner = new NavSideTRTRunner(encoder_path, policy_path);
    } catch (const std::exception& exc) {
        PyErr_SetString(PyExc_RuntimeError, exc.what());
        return -1;
    }
    return 0;
}

void NavSideTRTRunner_dealloc(PyNavSideTRTRunnerObject* self) {
    delete self->runner;
    self->runner = nullptr;
    Py_TYPE(self)->tp_free(reinterpret_cast<PyObject*>(self));
}

PyObject* NavSideTRTRunner_encode_depth(PyObject* self, PyObject* arg) {
    auto* runner_obj = require_runner(self);
    if (runner_obj == nullptr) {
        return nullptr;
    }
    PyArrayObject* depth = require_array(arg, "depth_tensor");
    if (depth == nullptr) {
        return nullptr;
    }
    try {
        return runner_obj->runner->encode_depth(depth);
    } catch (const std::exception& exc) {
        return raise_runtime_error(exc);
    }
}

PyObject* NavSideTRTRunner_run_policy(PyObject* self, PyObject* args) {
    auto* runner_obj = require_runner(self);
    if (runner_obj == nullptr) {
        return nullptr;
    }
    PyObject* obs_obj = nullptr;
    PyObject* h_in_obj = nullptr;
    PyObject* c_in_obj = nullptr;
    if (!PyArg_ParseTuple(args, "OOO", &obs_obj, &h_in_obj, &c_in_obj)) {
        return nullptr;
    }
    PyArrayObject* obs = require_array(obs_obj, "obs");
    PyArrayObject* h_in = require_array(h_in_obj, "h_in");
    PyArrayObject* c_in = require_array(c_in_obj, "c_in");
    if (obs == nullptr || h_in == nullptr || c_in == nullptr) {
        return nullptr;
    }
    try {
        return runner_obj->runner->run_policy(obs, h_in, c_in);
    } catch (const std::exception& exc) {
        return raise_runtime_error(exc);
    }
}

PyMethodDef NavSideTRTRunner_methods[] = {
    {"encode_depth", reinterpret_cast<PyCFunction>(NavSideTRTRunner_encode_depth), METH_O, "Run the encoder engine."},
    {"run_policy", reinterpret_cast<PyCFunction>(NavSideTRTRunner_run_policy), METH_VARARGS, "Run the policy engine."},
    {nullptr, nullptr, 0, nullptr},
};

PyTypeObject NavSideTRTRunnerType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
};

PyModuleDef module_def = {
    PyModuleDef_HEAD_INIT,
    "navside_trt",
    "NavSide TensorRT inference extension",
    -1,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

}  // namespace

PyMODINIT_FUNC PyInit_navside_trt(void) {
    import_array();

    NavSideTRTRunnerType.tp_name = "navside_trt.NavSideTRTRunner";
    NavSideTRTRunnerType.tp_basicsize = sizeof(PyNavSideTRTRunnerObject);
    NavSideTRTRunnerType.tp_itemsize = 0;
    NavSideTRTRunnerType.tp_dealloc = reinterpret_cast<destructor>(NavSideTRTRunner_dealloc);
    NavSideTRTRunnerType.tp_flags = Py_TPFLAGS_DEFAULT;
    NavSideTRTRunnerType.tp_doc = "NavSide TensorRT runner";
    NavSideTRTRunnerType.tp_methods = NavSideTRTRunner_methods;
    NavSideTRTRunnerType.tp_init = reinterpret_cast<initproc>(NavSideTRTRunner_init);
    NavSideTRTRunnerType.tp_new = NavSideTRTRunner_new;

    if (PyType_Ready(&NavSideTRTRunnerType) < 0) {
        return nullptr;
    }

    PyObject* module = PyModule_Create(&module_def);
    if (module == nullptr) {
        return nullptr;
    }

    Py_INCREF(&NavSideTRTRunnerType);
    if (PyModule_AddObject(module, "NavSideTRTRunner", reinterpret_cast<PyObject*>(&NavSideTRTRunnerType)) < 0) {
        Py_DECREF(&NavSideTRTRunnerType);
        Py_DECREF(module);
        return nullptr;
    }

    return module;
}
