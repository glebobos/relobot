#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cmath>
#include <fields2cover/path_planning/dubins_curves.h>

static PyObject * dubins(PyObject *, PyObject * args)
{
  double start_x, start_y, start_yaw, end_x, end_y, end_yaw, radius, spacing;
  if (!PyArg_ParseTuple(args, "(ddd)(ddd)dd", &start_x, &start_y, &start_yaw,
      &end_x, &end_y, &end_yaw, &radius, &spacing))
  {
    return nullptr;
  }
  for (double value : {start_x, start_y, start_yaw, end_x, end_y, end_yaw, radius, spacing}) {
    if (!std::isfinite(value)) {
      PyErr_SetString(PyExc_ValueError, "Dubins inputs must be finite");
      return nullptr;
    }
  }
  if (radius <= 0.0 || spacing < 0.005 || spacing > 0.1 || radius > 10.0 ||
      std::hypot(end_x - start_x, end_y - start_y) > 100.0)
  {
    PyErr_SetString(PyExc_ValueError, "Dubins geometry exceeds supported bounds");
    return nullptr;
  }
  try {
    F2CRobot robot;
    robot.setMinRadius(radius);
    f2c::pp::DubinsCurves generator;
    generator.using_cache = false;
    generator.discretization = spacing;
    const auto path = generator.createTurn(robot, F2CPoint(start_x, start_y), start_yaw,
      F2CPoint(end_x, end_y), end_yaw);
    PyObject * result = PyList_New(path.size());
    if (!result) {return nullptr;}
    for (size_t index = 0; index < path.size(); ++index) {
      const auto & state = path.states[index];
      PyObject * pose = Py_BuildValue("(ddd)", state.point.getX(), state.point.getY(), state.angle);
      if (!pose) {
        Py_DECREF(result);
        return nullptr;
      }
      PyList_SET_ITEM(result, index, pose);
    }
    return result;
  } catch (const std::exception & error) {
    PyErr_SetString(PyExc_ValueError, error.what());
    return nullptr;
  }
}

static PyMethodDef methods[] = {
  {"dubins", dubins, METH_VARARGS, "Sample a forward Fields2Cover Dubins connection."},
  {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "_coverage_geometry", nullptr, -1, methods};
PyMODINIT_FUNC PyInit__coverage_geometry() {return PyModule_Create(&module);}