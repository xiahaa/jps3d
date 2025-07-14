#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "BL_JPS.h"
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(BL_JPS, m) {
	m.doc() = "Python bindings for the C++ implementation of BL_JPS";

	py::class_<BL_JPS>(m, "BL_JPS")
		.def(py::init<>(), "Construct a pathfinder")
		.def("flushReProcess", &BL_JPS::flushReProcess, "")
		.def("preProcessGrid", &BL_JPS::preProcessGrid, "", "grid"_a, "width"_a, "height"_a)
		.def("reProcessGrid", &BL_JPS::reProcessGrid, "", "lx"_a, "rx"_a, "ty"_a, "by"_a)

		.def("findSolution", &BL_JPS::findSolution, "", "sX"_a, "sY"_a, "eX"_a, "eY"_a)
		.def("plan_2d", &BL_JPS::plan_2d, "", "grid"_a, "width"_a, "height"_a, "startX"_a, "startY"_a, "endX"_a, "endY"_a, "originX"_a, "originY"_a, "resolution"_a);

	// Define the PlanResult struct for Python
    py::class_<PlanResult>(m, "PlanResult")
        .def(py::init<>())
        .def_readwrite("path", &PlanResult::path)
        .def_readwrite("time_spent", &PlanResult::time_spent);

	py::class_<Coordinate>(m, "Coordinate")
		.def(py::init<>())
		.def(py::init<short, short>())
		.def("dist", &Coordinate::dist, "", "rhs"_a)
		.def("distSqrt", &Coordinate::distSqrt, "", "rhs"_a)
		.def("add", &Coordinate::add, "", "rhs"_a)
		.def_readwrite("x", &Coordinate::x)
		.def_readwrite("y", &Coordinate::y)
		.def("__getitem__", [](const Coordinate& c, size_t i) {
			if (i < 0 || i > 1) throw py::index_error("Index out of bounds");
			return c[i];
			})
		.def("__setitem__", [](Coordinate& c, size_t i, float v) {
				if (i < 0 || i > 1) throw py::index_error("Index out of bounds");
				c[i] = v;
			})
		.def("__eq__", &Coordinate::operator==, py::is_operator())
		.def("__neq__", &Coordinate::operator!=, py::is_operator());
}
