// pybind11 module entry point for libpyfranka.
//
// Wraps franka::Robot plus our four streaming controllers so they can be
// driven from Python.  Robot construction may block (network handshake) so
// we release the GIL around it.  Per-cycle target setters and state getters
// are short, mutex-protected operations and do NOT release the GIL.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "realtime_control/cartesian_impedance_control.hpp"
#include "realtime_control/cartesian_pose_control.hpp"
#include "realtime_control/cartesian_state.hpp"
#include "realtime_control/joint_position_control.hpp"
#include "realtime_control/joint_state.hpp"
#include "realtime_control/joint_torque_control.hpp"
#include "realtime_control/logging.hpp"

namespace py = pybind11;
using namespace franka_rt;

// ---- Helpers: convert numpy ↔ std::array ----

template <size_t N>
static std::array<double, N> NumpyToArray(py::array_t<double> arr) {
    if (static_cast<size_t>(arr.size()) != N) {
        throw std::invalid_argument("expected array of size " + std::to_string(N));
    }
    std::array<double, N> out;
    auto r = arr.unchecked();
    if (arr.ndim() == 1) {
        for (size_t i = 0; i < N; ++i) out[i] = arr.at(i);
    } else {
        // Flatten any shape in C order
        auto flat = arr.attr("flatten")().cast<py::array_t<double>>();
        for (size_t i = 0; i < N; ++i) out[i] = flat.at(i);
    }
    return out;
}

// 4×4 row-major numpy → 16-element column-major std::array (libfranka format).
static std::array<double, 16> NumpyMat4ToColMajor(py::array arr) {
    auto a = py::array_t<double>::ensure(arr);
    if (!a) throw std::invalid_argument("pose must be a numeric array");
    if (a.ndim() != 2 || a.shape(0) != 4 || a.shape(1) != 4) {
        throw std::invalid_argument("pose must be a 4x4 matrix");
    }
    std::array<double, 16> out;
    auto r = a.unchecked<2>();
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            out[c * 4 + row] = r(row, c);
        }
    }
    return out;
}

// 16-element column-major std::array → 4×4 row-major numpy.
static py::array_t<double> ColMajorToNumpyMat4(const std::array<double, 16>& T) {
    py::array_t<double> arr({4, 4});
    auto r = arr.mutable_unchecked<2>();
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            r(row, c) = T[c * 4 + row];
        }
    }
    return arr;
}

template <size_t N>
static py::array_t<double> ArrayToNumpy(const std::array<double, N>& a) {
    py::array_t<double> arr(N);
    auto r = arr.mutable_unchecked<1>();
    for (size_t i = 0; i < N; ++i) r(i) = a[i];
    return arr;
}

// ---- Python-facing Robot wrapper ----

class PyRobot {
public:
    PyRobot(const std::string& fci_address,
            const std::string& realtime_config = "enforce") {
        auto rtc = (realtime_config == "ignore")
                       ? franka::RealtimeConfig::kIgnore
                       : franka::RealtimeConfig::kEnforce;
        py::gil_scoped_release nogil;
        robot_ = std::make_unique<franka::Robot>(fci_address, rtc);
    }

    franka::Robot& raw() { return *robot_; }

    py::dict readOnce() {
        franka::RobotState state;
        {
            py::gil_scoped_release nogil;
            state = robot_->readOnce();
        }
        py::dict d;
        d["q"]                       = ArrayToNumpy(state.q);
        d["dq"]                      = ArrayToNumpy(state.dq);
        d["tau_J"]                   = ArrayToNumpy(state.tau_J);
        d["tau_ext_hat_filtered"]    = ArrayToNumpy(state.tau_ext_hat_filtered);
        d["O_T_EE"]                  = ColMajorToNumpyMat4(state.O_T_EE);
        d["O_F_ext_hat_K"]           = ArrayToNumpy(state.O_F_ext_hat_K);
        d["K_F_ext_hat_K"]           = ArrayToNumpy(state.K_F_ext_hat_K);
        d["control_command_success_rate"] = state.control_command_success_rate;
        return d;
    }

    void setCollisionBehavior(py::array_t<double> lower_torque,
                              py::array_t<double> upper_torque,
                              py::array_t<double> lower_force,
                              py::array_t<double> upper_force) {
        auto lt = NumpyToArray<7>(lower_torque);
        auto ut = NumpyToArray<7>(upper_torque);
        auto lf = NumpyToArray<6>(lower_force);
        auto uf = NumpyToArray<6>(upper_force);
        py::gil_scoped_release nogil;
        robot_->setCollisionBehavior(lt, ut, lf, uf);
    }

    void setJointImpedance(py::array_t<double> K) {
        auto k = NumpyToArray<7>(K);
        py::gil_scoped_release nogil;
        robot_->setJointImpedance(k);
    }

    void setCartesianImpedance(py::array_t<double> K) {
        auto k = NumpyToArray<6>(K);
        py::gil_scoped_release nogil;
        robot_->setCartesianImpedance(k);
    }

    void automaticErrorRecovery() {
        py::gil_scoped_release nogil;
        robot_->automaticErrorRecovery();
    }

    void stop() {
        py::gil_scoped_release nogil;
        robot_->stop();
    }

private:
    std::unique_ptr<franka::Robot> robot_;
};

// ---- Controller wrappers ----

class PyJointPositionControl {
public:
    PyJointPositionControl(PyRobot& robot, double max_step)
        : ctrl_(robot.raw(), max_step) {}
    void start()   { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()    { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetJoints(py::array_t<double> q) {
        ctrl_.setTargetJoints(NumpyToArray<7>(q));
    }
    py::dict getState() const {
        auto s = ctrl_.getState();
        py::dict d;
        d["q"] = ArrayToNumpy(s.q);
        d["dq"] = ArrayToNumpy(s.dq);
        d["tau_J"] = ArrayToNumpy(s.tau_J);
        d["tau_ext_hat_filtered"] = ArrayToNumpy(s.tau_ext_hat_filtered);
        d["O_T_EE"] = ColMajorToNumpyMat4(s.O_T_EE);
        d["control_command_success_rate"] = s.control_command_success_rate;
        return d;
    }
    void moveToJoints(py::array_t<double> q, double duration_sec) {
        ctrl_.moveToJoints(NumpyToArray<7>(q), duration_sec);
    }
    bool isMoving() const { return ctrl_.isMoving(); }
    void cancelMove() { ctrl_.cancelMove(); }

private:
    JointPositionControl ctrl_;
};

class PyJointTorqueControl {
public:
    explicit PyJointTorqueControl(PyRobot& robot) : ctrl_(robot.raw()) {}
    void start()   { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()    { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetTorques(py::array_t<double> tau) {
        ctrl_.setTargetTorques(NumpyToArray<7>(tau));
    }
    py::dict getState() const {
        auto s = ctrl_.getState();
        py::dict d;
        d["q"] = ArrayToNumpy(s.q);
        d["dq"] = ArrayToNumpy(s.dq);
        d["tau_J"] = ArrayToNumpy(s.tau_J);
        d["tau_ext_hat_filtered"] = ArrayToNumpy(s.tau_ext_hat_filtered);
        d["O_T_EE"] = ColMajorToNumpyMat4(s.O_T_EE);
        d["control_command_success_rate"] = s.control_command_success_rate;
        return d;
    }

private:
    JointTorqueControl ctrl_;
};

class PyCartesianPoseControl {
public:
    PyCartesianPoseControl(PyRobot& robot, double max_t_step, double max_r_step)
        : ctrl_(robot.raw(), max_t_step, max_r_step) {}
    void start()   { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()    { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetPose(py::array T) {
        ctrl_.setTargetPose(NumpyMat4ToColMajor(T));
    }
    py::dict getState() const {
        auto s = ctrl_.getState();
        py::dict d;
        d["O_T_EE"] = ColMajorToNumpyMat4(s.O_T_EE);
        d["O_dP_EE_d"] = ArrayToNumpy(s.O_dP_EE_d);
        d["K_F_ext_hat_K"] = ArrayToNumpy(s.K_F_ext_hat_K);
        d["O_F_ext_hat_K"] = ArrayToNumpy(s.O_F_ext_hat_K);
        d["q"] = ArrayToNumpy(s.q);
        d["dq"] = ArrayToNumpy(s.dq);
        d["tau_ext_hat_filtered"] = ArrayToNumpy(s.tau_ext_hat_filtered);
        d["control_command_success_rate"] = s.control_command_success_rate;
        return d;
    }
    void moveToPose(py::array T, double duration_sec) {
        ctrl_.moveToPose(NumpyMat4ToColMajor(T), duration_sec);
    }
    bool isMoving() const { return ctrl_.isMoving(); }
    void cancelMove() { ctrl_.cancelMove(); }

private:
    CartesianPoseControl ctrl_;
};

class PyCartesianImpedanceControl {
public:
    explicit PyCartesianImpedanceControl(PyRobot& robot) : ctrl_(robot.raw()) {}
    void start()   { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()    { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetPose(py::array T) {
        ctrl_.setTargetPose(NumpyMat4ToColMajor(T));
    }
    void setStiffness(py::array_t<double> K) {
        ctrl_.setStiffness(NumpyToArray<6>(K));
    }
    void setDamping(py::array_t<double> D) {
        ctrl_.setDamping(NumpyToArray<6>(D));
    }
    py::dict getState() const {
        auto s = ctrl_.getState();
        py::dict d;
        d["O_T_EE"] = ColMajorToNumpyMat4(s.O_T_EE);
        d["O_dP_EE_d"] = ArrayToNumpy(s.O_dP_EE_d);
        d["K_F_ext_hat_K"] = ArrayToNumpy(s.K_F_ext_hat_K);
        d["O_F_ext_hat_K"] = ArrayToNumpy(s.O_F_ext_hat_K);
        d["q"] = ArrayToNumpy(s.q);
        d["dq"] = ArrayToNumpy(s.dq);
        d["tau_ext_hat_filtered"] = ArrayToNumpy(s.tau_ext_hat_filtered);
        d["control_command_success_rate"] = s.control_command_success_rate;
        return d;
    }

private:
    CartesianImpedanceControl ctrl_;
};

PYBIND11_MODULE(_franka_rt, m) {
    m.doc() = "libpyfranka — 1 kHz Python real-time control for Franka FR3.";

    py::register_exception<franka::Exception>(m, "FrankaException");
    py::register_exception<franka::ControlException>(m, "ControlException");
    py::register_exception<franka::CommandException>(m, "CommandException");
    py::register_exception<franka::NetworkException>(m, "NetworkException");

    py::class_<PyRobot>(m, "Robot")
        .def(py::init<const std::string&, const std::string&>(),
             py::arg("fci_address"), py::arg("realtime_config") = "enforce")
        .def("read_once", &PyRobot::readOnce)
        .def("set_collision_behavior", &PyRobot::setCollisionBehavior,
             py::arg("lower_torque_thresholds"),
             py::arg("upper_torque_thresholds"),
             py::arg("lower_force_thresholds"),
             py::arg("upper_force_thresholds"))
        .def("set_joint_impedance", &PyRobot::setJointImpedance, py::arg("K"))
        .def("set_cartesian_impedance", &PyRobot::setCartesianImpedance, py::arg("K"))
        .def("automatic_error_recovery", &PyRobot::automaticErrorRecovery)
        .def("stop", &PyRobot::stop)
        .def("__enter__", [](PyRobot& self) -> PyRobot& { return self; })
        .def("__exit__", [](PyRobot& self, py::object, py::object, py::object) {
            self.stop();
        })
        .def("start_joint_position_control",
             [](PyRobot& self, double max_step) {
                 auto p = std::make_unique<PyJointPositionControl>(self, max_step);
                 p->start();
                 return p;
             },
             py::arg("max_joint_step_per_cycle") = 0.001,
             py::keep_alive<0, 1>())
        .def("start_joint_torque_control",
             [](PyRobot& self) {
                 auto p = std::make_unique<PyJointTorqueControl>(self);
                 p->start();
                 return p;
             },
             py::keep_alive<0, 1>())
        .def("start_cartesian_pose_control",
             [](PyRobot& self, double max_t, double max_r) {
                 auto p = std::make_unique<PyCartesianPoseControl>(self, max_t, max_r);
                 p->start();
                 return p;
             },
             py::arg("max_translational_step") = 0.001,
             py::arg("max_rotational_step")    = 0.005,
             py::keep_alive<0, 1>())
        .def("start_cartesian_impedance_control",
             [](PyRobot& self) {
                 auto p = std::make_unique<PyCartesianImpedanceControl>(self);
                 p->start();
                 return p;
             },
             py::keep_alive<0, 1>());

    py::class_<PyJointPositionControl>(m, "JointPositionControl")
        .def("set_target_joints", &PyJointPositionControl::setTargetJoints, py::arg("q"))
        .def("get_state", &PyJointPositionControl::getState)
        .def("move_to_joints", &PyJointPositionControl::moveToJoints,
             py::arg("q"), py::arg("duration_sec") = 0.0)
        .def("is_moving", &PyJointPositionControl::isMoving)
        .def("cancel_move", &PyJointPositionControl::cancelMove)
        .def("trigger_estop", &PyJointPositionControl::triggerEstop)
        .def("stop", &PyJointPositionControl::stop)
        .def("is_running", &PyJointPositionControl::isRunning)
        .def("__enter__", [](PyJointPositionControl& s) -> PyJointPositionControl& { return s; })
        .def("__exit__", [](PyJointPositionControl& s, py::object, py::object, py::object) { s.stop(); });

    py::class_<PyJointTorqueControl>(m, "JointTorqueControl")
        .def("set_target_torques", &PyJointTorqueControl::setTargetTorques, py::arg("tau"))
        .def("get_state", &PyJointTorqueControl::getState)
        .def("trigger_estop", &PyJointTorqueControl::triggerEstop)
        .def("stop", &PyJointTorqueControl::stop)
        .def("is_running", &PyJointTorqueControl::isRunning)
        .def("__enter__", [](PyJointTorqueControl& s) -> PyJointTorqueControl& { return s; })
        .def("__exit__", [](PyJointTorqueControl& s, py::object, py::object, py::object) { s.stop(); });

    py::class_<PyCartesianPoseControl>(m, "CartesianPoseControl")
        .def("set_target_pose", &PyCartesianPoseControl::setTargetPose, py::arg("O_T_EE"))
        .def("get_state", &PyCartesianPoseControl::getState)
        .def("move_to_pose", &PyCartesianPoseControl::moveToPose,
             py::arg("O_T_EE"), py::arg("duration_sec") = 0.0)
        .def("is_moving", &PyCartesianPoseControl::isMoving)
        .def("cancel_move", &PyCartesianPoseControl::cancelMove)
        .def("trigger_estop", &PyCartesianPoseControl::triggerEstop)
        .def("stop", &PyCartesianPoseControl::stop)
        .def("is_running", &PyCartesianPoseControl::isRunning)
        .def("__enter__", [](PyCartesianPoseControl& s) -> PyCartesianPoseControl& { return s; })
        .def("__exit__", [](PyCartesianPoseControl& s, py::object, py::object, py::object) { s.stop(); });

    py::class_<PyCartesianImpedanceControl>(m, "CartesianImpedanceControl")
        .def("set_target_pose", &PyCartesianImpedanceControl::setTargetPose, py::arg("O_T_EE"))
        .def("set_stiffness", &PyCartesianImpedanceControl::setStiffness, py::arg("K"))
        .def("set_damping", &PyCartesianImpedanceControl::setDamping, py::arg("D"))
        .def("get_state", &PyCartesianImpedanceControl::getState)
        .def("trigger_estop", &PyCartesianImpedanceControl::triggerEstop)
        .def("stop", &PyCartesianImpedanceControl::stop)
        .def("is_running", &PyCartesianImpedanceControl::isRunning)
        .def("__enter__", [](PyCartesianImpedanceControl& s) -> PyCartesianImpedanceControl& { return s; })
        .def("__exit__", [](PyCartesianImpedanceControl& s, py::object, py::object, py::object) { s.stop(); });
}
