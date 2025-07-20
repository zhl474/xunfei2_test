#include <pybind11/pybind11.h>

#ifndef PYBIND11_MODULE
#error "Pybind11 module macro not defined!"
#endif

PYBIND11_MODULE(test_m, m) {
    m.def("hello", []() { return "Hello, World!"; });
}