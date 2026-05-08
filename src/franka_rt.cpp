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
#include <franka/errors.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "realtime_control/cartesian_control.hpp"
#include "realtime_control/cartesian_state.hpp"
#include "realtime_control/joint_position_control.hpp"
#include "realtime_control/joint_state.hpp"
#include "realtime_control/logging.hpp"

namespace py = pybind11;
using namespace franka_rt;

// ---- libfranka-state translation helpers ----

static RobotMode ConvertRobotMode(franka::RobotMode m) {
    switch (m) {
        case franka::RobotMode::kIdle:                    return RobotMode::Idle;
        case franka::RobotMode::kMove:                    return RobotMode::Move;
        case franka::RobotMode::kGuiding:                 return RobotMode::Guiding;
        case franka::RobotMode::kReflex:                  return RobotMode::Reflex;
        case franka::RobotMode::kUserStopped:             return RobotMode::UserStopped;
        case franka::RobotMode::kAutomaticErrorRecovery:  return RobotMode::AutomaticErrorRecovery;
        default:                                          return RobotMode::Other;
    }
}

// ---- RealtimeConfig wrapper (mirrors pylibfranka.RealtimeConfig) ----
enum class PyRealtimeConfig : uint8_t { kEnforce = 0, kIgnore = 1 };

// ---- ControllerMode wrapper (mirrors pylibfranka.ControllerMode) ----
//
// pylibfranka exposes these as JointImpedance / CartesianImpedance (no k
// prefix), unlike RealtimeConfig.kEnforce / kIgnore.  We mirror that.
enum class PyControllerMode : uint8_t { JointImpedance = 0, CartesianImpedance = 1 };

inline research_interface::robot::Move::ControllerMode ToFrankaControllerMode(
        PyControllerMode m) {
    return (m == PyControllerMode::CartesianImpedance)
        ? research_interface::robot::Move::ControllerMode::kCartesianImpedance
        : research_interface::robot::Move::ControllerMode::kJointImpedance;
}

// libfranka's franka::Errors has 41 fields; operator std::string() returns
// "[name1, name2, ...]" with active flags only.  We pass that through to
// Python as a single string for cheap display, plus a bool for quick
// "any error?" checks.
static std::string ErrorsToString(const franka::Errors& e) {
    return static_cast<std::string>(e);
}

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
            PyRealtimeConfig realtime_config = PyRealtimeConfig::kIgnore,
            bool disable_default_behavior = false,
            double load_mass = 0.0,
            py::object load_com = py::none(),
            py::object load_inertia = py::none()) {
        // Default to kIgnore because Franka's kEnforce path tries mlockall
        // which OOMs Python+PyTorch processes.  Users can opt in to kEnforce.
        auto rtc = (realtime_config == PyRealtimeConfig::kEnforce)
                       ? franka::RealtimeConfig::kEnforce
                       : franka::RealtimeConfig::kIgnore;
        {
            py::gil_scoped_release nogil;
            robot_ = std::make_unique<franka::Robot>(fci_address, rtc);
        }
        if (!disable_default_behavior) {
            applyDefaultBehavior();
        }
        // If user told us about an attached end-effector, register it now.
        if (load_mass > 0.0) {
            std::array<double, 3> com = {0.0, 0.0, 0.0};
            std::array<double, 9> inertia = {0,0,0, 0,0,0, 0,0,0};
            if (!load_com.is_none()) {
                com = NumpyToArray<3>(load_com.cast<py::array_t<double>>());
            }
            if (!load_inertia.is_none()) {
                inertia = NumpyToArray<9>(load_inertia.cast<py::array_t<double>>());
            } else {
                // Tiny default inertia so libfranka's filter doesn't NaN-out.
                // 1/600 of a 0.1m × 0.1m × 0.1m solid cube of mass=load_mass.
                double i_diag = load_mass * 0.01 * 0.01 / 6.0;
                inertia = {i_diag, 0, 0,  0, i_diag, 0,  0, 0, i_diag};
            }
            py::gil_scoped_release nogil;
            robot_->setLoad(load_mass, com, inertia);
        }
    }

    ~PyRobot() {
        close();
    }

    franka::Robot& raw() { return *robot_; }

    py::dict readOnce() {
        franka::RobotState state;
        {
            py::gil_scoped_release nogil;
            state = robot_->readOnce();
        }
        return buildStateDict(state);
    }

    py::dict buildStateDict(const franka::RobotState& state) const {
        py::dict d;
        // Measured
        d["q"]                       = ArrayToNumpy(state.q);
        d["dq"]                      = ArrayToNumpy(state.dq);
        d["tau_J"]                   = ArrayToNumpy(state.tau_J);
        d["dtau_J"]                  = ArrayToNumpy(state.dtau_J);
        d["tau_ext_hat_filtered"]    = ArrayToNumpy(state.tau_ext_hat_filtered);
        d["O_T_EE"]                  = ColMajorToNumpyMat4(state.O_T_EE);
        d["elbow"]                   = ArrayToNumpy(state.elbow);
        // Last desired (output of internal motion gen)
        d["q_d"]                     = ArrayToNumpy(state.q_d);
        d["dq_d"]                    = ArrayToNumpy(state.dq_d);
        d["ddq_d"]                   = ArrayToNumpy(state.ddq_d);
        d["tau_J_d"]                 = ArrayToNumpy(state.tau_J_d);
        d["O_T_EE_d"]                = ColMajorToNumpyMat4(state.O_T_EE_d);
        d["O_dP_EE_d"]               = ArrayToNumpy(state.O_dP_EE_d);
        d["elbow_d"]                 = ArrayToNumpy(state.elbow_d);
        // Last commanded
        d["O_T_EE_c"]                = ColMajorToNumpyMat4(state.O_T_EE_c);
        d["O_dP_EE_c"]               = ArrayToNumpy(state.O_dP_EE_c);
        d["O_ddP_EE_c"]              = ArrayToNumpy(state.O_ddP_EE_c);
        d["elbow_c"]                 = ArrayToNumpy(state.elbow_c);
        // External wrench
        d["O_F_ext_hat_K"]           = ArrayToNumpy(state.O_F_ext_hat_K);
        d["K_F_ext_hat_K"]           = ArrayToNumpy(state.K_F_ext_hat_K);
        // Status / errors
        d["current_errors"]          = ErrorsToString(state.current_errors);
        d["has_error"]               = static_cast<bool>(state.current_errors);
        d["last_motion_errors"]      = ErrorsToString(state.last_motion_errors);
        d["robot_mode"]              = ConvertRobotMode(state.robot_mode);
        d["control_command_success_rate"] = state.control_command_success_rate;
        return d;
    }

    // ---- NRT command APIs ----
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

    void setLoad(double load_mass,
                 py::array_t<double> F_x_Cload,
                 py::array_t<double> load_inertia) {
        auto fxc = NumpyToArray<3>(F_x_Cload);
        auto in  = NumpyToArray<9>(load_inertia);
        py::gil_scoped_release nogil;
        robot_->setLoad(load_mass, fxc, in);
    }

    void setEE(py::array_t<double> NE_T_EE) {
        auto t = NumpyToArray<16>(NE_T_EE);
        py::gil_scoped_release nogil;
        robot_->setEE(t);
    }

    void setK(py::array_t<double> EE_T_K) {
        auto t = NumpyToArray<16>(EE_T_K);
        py::gil_scoped_release nogil;
        robot_->setK(t);
    }

    // Position-dependent joint velocity limits (NRT; do not call from RT loop).
    py::array_t<double> getUpperJointVelocityLimits(py::array_t<double> q) {
        auto qa = NumpyToArray<7>(q);
        std::array<double, 7> r;
        {
            py::gil_scoped_release nogil;
            r = robot_->getUpperJointVelocityLimits(qa);
        }
        return ArrayToNumpy(r);
    }

    py::array_t<double> getLowerJointVelocityLimits(py::array_t<double> q) {
        auto qa = NumpyToArray<7>(q);
        std::array<double, 7> r;
        {
            py::gil_scoped_release nogil;
            r = robot_->getLowerJointVelocityLimits(qa);
        }
        return ArrayToNumpy(r);
    }

    // Idempotent close: safe to call multiple times.  Used by both
    // explicit stop() and the destructor / __exit__.
    void close() {
        if (closed_.exchange(true)) return;
        if (robot_) {
            try {
                py::gil_scoped_release nogil;
                robot_->stop();
            } catch (...) {
                // Robot may already be in a bad state; swallow to avoid
                // throwing from destructor / __exit__.
            }
        }
    }

    void stop() { close(); }

private:
    // Set up Franka behavior with collision thresholds high enough to allow
    // teleoperation pushes without spurious cartesian_reflex.  These match
    // the libfranka official cartesian_impedance_control.cpp example which
    // explicitly raises thresholds to 100 N / 100 Nm before running an
    // impedance controller, with the warning:
    //
    //   "WARNING: Collision thresholds are set to high values.
    //    Make sure you have the user stop button at hand!"
    //
    // The user-stop button is the safety boundary; software thresholds at
    // 100 N just prevent the robot from over-reacting to normal contact
    // forces during compliance testing or teleop.
    void applyDefaultBehavior() {
        std::array<double, 7> torq_low =  {100, 100, 100, 100, 100, 100, 100};
        std::array<double, 7> torq_high = {100, 100, 100, 100, 100, 100, 100};
        std::array<double, 6> force_low  = {100, 100, 100, 100, 100, 100};
        std::array<double, 6> force_high = {100, 100, 100, 100, 100, 100};
        std::array<double, 7> joint_imp = {3000, 3000, 3000, 2500, 2500, 2000, 2000};
        std::array<double, 6> cart_imp =  {3000, 3000, 3000, 300, 300, 300};
        py::gil_scoped_release nogil;
        robot_->setCollisionBehavior(torq_low, torq_high, force_low, force_high);
        robot_->setJointImpedance(joint_imp);
        robot_->setCartesianImpedance(cart_imp);
    }

    std::unique_ptr<franka::Robot> robot_;
    std::atomic<bool> closed_{false};
};

// ---- Controller wrappers ----

class PyJointPositionControl {
public:
    explicit PyJointPositionControl(PyRobot& robot)
        : ctrl_(robot.raw()) {}
    void start()   { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()    { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetJoints(py::array_t<double> q) {
        ctrl_.setTargetJoints(NumpyToArray<7>(q));
    }
    py::dict getState() const {
        // Release GIL across the C++ struct copy so other Python threads
        // are not blocked even if the RT thread happens to hold the
        // mutex (~microsecond contention).  Re-acquire (implicit) before
        // building the py::dict, since pybind needs the GIL for that.
        JointState s;
        {
            py::gil_scoped_release nogil;
            s = ctrl_.getState();
        }
        py::dict d;
        d["q"]                      = ArrayToNumpy(s.q);
        d["dq"]                     = ArrayToNumpy(s.dq);
        d["tau_J"]                  = ArrayToNumpy(s.tau_J);
        d["dtau_J"]                 = ArrayToNumpy(s.dtau_J);
        d["tau_ext_hat_filtered"]   = ArrayToNumpy(s.tau_ext_hat_filtered);
        d["O_T_EE"]                 = ColMajorToNumpyMat4(s.O_T_EE);
        d["q_d"]                    = ArrayToNumpy(s.q_d);
        d["dq_d"]                   = ArrayToNumpy(s.dq_d);
        d["ddq_d"]                  = ArrayToNumpy(s.ddq_d);
        d["tau_J_d"]                = ArrayToNumpy(s.tau_J_d);
        d["O_F_ext_hat_K"]          = ArrayToNumpy(s.O_F_ext_hat_K);
        d["K_F_ext_hat_K"]          = ArrayToNumpy(s.K_F_ext_hat_K);
        d["current_errors"]         = s.current_errors;
        d["last_motion_errors"]     = s.last_motion_errors;
        d["robot_mode"]             = s.robot_mode;
        d["has_error"]              = (s.current_errors != 0ULL);
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

// Maps the public PyControllerMode enum (mirrors pylibfranka.ControllerMode)
// to our internal franka_rt::ControllerMode used by CartesianControl.
inline ControllerMode ToInternalControllerMode(PyControllerMode m) {
    return (m == PyControllerMode::CartesianImpedance)
        ? ControllerMode::CartesianImpedance
        : ControllerMode::JointImpedance;
}

class PyCartesianControl {
public:
    PyCartesianControl(PyRobot& robot, PyControllerMode mode)
        : ctrl_(robot.raw(), ToInternalControllerMode(mode)) {}
    void start()        { py::gil_scoped_release nogil; ctrl_.start(); }
    void stop()         { py::gil_scoped_release nogil; ctrl_.stop(); }
    bool isRunning() const { return ctrl_.isRunning(); }
    void triggerEstop() { ctrl_.triggerEstop(); }

    void setTargetPose(py::array T) {
        ctrl_.setTargetPose(NumpyMat4ToColMajor(T));
    }
    void setTargetPoseWithElbow(py::array T, py::array_t<double> elbow) {
        ctrl_.setTargetPoseWithElbow(
            NumpyMat4ToColMajor(T),
            NumpyToArray<2>(elbow));
    }
    py::dict getState() const {
        // GIL released around the mutex-protected struct copy.  This
        // means a Python thread polling get_state() at any rate will
        // never block other Python threads, and the RT loop's mutex
        // hold (a few-hundred-byte struct copy at 1 kHz) cannot back
        // up Python work either.
        CartesianState s;
        {
            py::gil_scoped_release nogil;
            s = ctrl_.getState();
        }
        py::dict d;
        // Measured
        d["O_T_EE"]              = ColMajorToNumpyMat4(s.O_T_EE);
        d["q"]                   = ArrayToNumpy(s.q);
        d["dq"]                  = ArrayToNumpy(s.dq);
        d["elbow"]               = ArrayToNumpy(s.elbow);
        // Last desired
        d["O_T_EE_d"]            = ColMajorToNumpyMat4(s.O_T_EE_d);
        d["O_dP_EE_d"]           = ArrayToNumpy(s.O_dP_EE_d);
        d["q_d"]                 = ArrayToNumpy(s.q_d);
        d["dq_d"]                = ArrayToNumpy(s.dq_d);
        d["elbow_d"]             = ArrayToNumpy(s.elbow_d);
        // Last commanded
        d["O_T_EE_c"]            = ColMajorToNumpyMat4(s.O_T_EE_c);
        d["O_dP_EE_c"]           = ArrayToNumpy(s.O_dP_EE_c);
        d["O_ddP_EE_c"]          = ArrayToNumpy(s.O_ddP_EE_c);
        d["elbow_c"]             = ArrayToNumpy(s.elbow_c);
        // Wrench
        d["O_F_ext_hat_K"]       = ArrayToNumpy(s.O_F_ext_hat_K);
        d["K_F_ext_hat_K"]       = ArrayToNumpy(s.K_F_ext_hat_K);
        d["tau_ext_hat_filtered"] = ArrayToNumpy(s.tau_ext_hat_filtered);
        // Status
        d["current_errors"]      = s.current_errors;       // bit-packed uint64
        d["last_motion_errors"]  = s.last_motion_errors;
        d["robot_mode"]          = s.robot_mode;
        d["has_error"]           = (s.current_errors != 0ULL);
        d["control_command_success_rate"] = s.control_command_success_rate;
        return d;
    }
    void moveToPose(py::array T, double duration_sec) {
        ctrl_.moveToPose(NumpyMat4ToColMajor(T), duration_sec);
    }
    bool isMoving() const { return ctrl_.isMoving(); }
    void cancelMove() { ctrl_.cancelMove(); }

private:
    CartesianControl ctrl_;
};

PYBIND11_MODULE(_franka_rt, m) {
    m.doc() = "libpyfranka — 1 kHz Python real-time control for Franka FR3.";

    py::register_exception<franka::Exception>(m, "FrankaException");
    py::register_exception<franka::ControlException>(m, "ControlException");
    py::register_exception<franka::CommandException>(m, "CommandException");
    py::register_exception<franka::NetworkException>(m, "NetworkException");

    // RobotMode enum mirrors pylibfranka.RobotMode (no k prefix).
    py::enum_<RobotMode>(m, "RobotMode")
        .value("Other",                   RobotMode::Other)
        .value("Idle",                    RobotMode::Idle)
        .value("Move",                    RobotMode::Move)
        .value("Guiding",                 RobotMode::Guiding)
        .value("Reflex",                  RobotMode::Reflex)
        .value("UserStopped",             RobotMode::UserStopped)
        .value("AutomaticErrorRecovery",  RobotMode::AutomaticErrorRecovery);

    // RealtimeConfig enum mirrors pylibfranka.RealtimeConfig (k prefix kept,
    // matching upstream).
    py::enum_<PyRealtimeConfig>(m, "RealtimeConfig")
        .value("kEnforce", PyRealtimeConfig::kEnforce)
        .value("kIgnore",  PyRealtimeConfig::kIgnore);

    // ControllerMode enum mirrors pylibfranka.ControllerMode (no k prefix).
    py::enum_<PyControllerMode>(m, "ControllerMode")
        .value("JointImpedance",     PyControllerMode::JointImpedance)
        .value("CartesianImpedance", PyControllerMode::CartesianImpedance);

    py::class_<PyRobot>(m, "Robot")
        .def(py::init<const std::string&, PyRealtimeConfig, bool,
                      double, py::object, py::object>(),
             py::arg("fci_address"),
             py::arg("realtime_config") = PyRealtimeConfig::kIgnore,
             py::arg("disable_default_behavior") = false,
             py::arg("load_mass") = 0.0,
             py::arg("load_com") = py::none(),
             py::arg("load_inertia") = py::none(),
             "Connect to the FCI.\n\n"
             "  realtime_config: RealtimeConfig.kEnforce or kIgnore.\n"
             "    Default kIgnore avoids libfranka's mlockall path which\n"
             "    can OOM Python processes that hold large numpy/torch tensors.\n"
             "  disable_default_behavior: skip the automatic call to\n"
             "    setCollisionBehavior + setJointImpedance + setCartesianImpedance.\n"
             "    Defaults to False (recommended for teleoperation).\n"
             "  load_mass: kg of any attached end-effector (e.g. gripper).\n"
             "    Equivalent to the 'tool' setting on a Flexiv teach pendant.\n"
             "    REQUIRED if the robot has anything mounted at the flange,\n"
             "    otherwise gravity compensation will be wrong and the IK can\n"
             "    produce joint discontinuities.  Set to 0 if no attachment.\n"
             "  load_com: 3-vector, payload COM in flange frame [m].\n"
             "  load_inertia: 9-elem 3x3 inertia tensor.  If omitted, a tiny\n"
             "    default is used.")
        .def("read_once", &PyRobot::readOnce,
             "Read one robot state synchronously.  Cannot be called while a\n"
             "control loop is running.")
        .def("set_collision_behavior", &PyRobot::setCollisionBehavior,
             py::arg("lower_torque_thresholds"),
             py::arg("upper_torque_thresholds"),
             py::arg("lower_force_thresholds"),
             py::arg("upper_force_thresholds"),
             "NRT command — call before start_*_control(), not from inside a\n"
             "control loop.")
        .def("set_joint_impedance", &PyRobot::setJointImpedance, py::arg("K"),
             "NRT command — sets gains for libfranka's internal joint\n"
             "impedance controller (used by start_joint_position_control and\n"
             "start_cartesian_pose_control(impedance_mode='joint')).")
        .def("set_cartesian_impedance", &PyRobot::setCartesianImpedance, py::arg("K"),
             "NRT command — sets 6D Cartesian stiffness for libfranka's\n"
             "internal Cartesian impedance controller.  Range: K_x[0:3] in\n"
             "[10, 3000] N/m, K_x[3:6] in [1, 300] Nm/rad.")
        .def("automatic_error_recovery", &PyRobot::automaticErrorRecovery)
        .def("set_load", &PyRobot::setLoad,
             py::arg("load_mass"),
             py::arg("F_x_Cload"),
             py::arg("load_inertia"),
             "NRT command — register the dynamic parameters of an attached\n"
             "end-effector / payload.  Without this call, the robot's gravity\n"
             "compensation is wrong and the IK / motion generator can produce\n"
             "joint-side discontinuities (joint_velocity_discontinuity).\n\n"
             "  load_mass: total mass of attached payload [kg]\n"
             "  F_x_Cload: 3-vector of payload center of mass in flange frame [m]\n"
             "  load_inertia: 9-element row-major 3×3 inertia tensor [kg·m²]")
        .def("set_EE", &PyRobot::setEE, py::arg("NE_T_EE"),
             "NRT command — set transformation from nominal end-effector to\n"
             "actual end-effector frame.  16-element column-major 4×4 matrix.")
        .def("set_K", &PyRobot::setK, py::arg("EE_T_K"),
             "NRT command — set transformation from end-effector to stiffness\n"
             "frame K (used by Cartesian impedance).  16-elem col-major 4×4.")
        .def("get_upper_joint_velocity_limits",
             &PyRobot::getUpperJointVelocityLimits, py::arg("q"),
             "Position-dependent upper joint velocity limits.  NRT — query\n"
             "before start_*_control(), not in the RT loop.")
        .def("get_lower_joint_velocity_limits",
             &PyRobot::getLowerJointVelocityLimits, py::arg("q"))
        .def("stop", &PyRobot::stop, "Idempotent — safe to call repeatedly.")
        .def("close", &PyRobot::close, "Alias for stop().")
        .def("__enter__", [](PyRobot& self) -> PyRobot& { return self; })
        .def("__exit__", [](PyRobot& self, py::object, py::object, py::object) {
            self.close();
        })
        .def("start_joint_position_control",
             [](PyRobot& self) {
                 auto p = std::make_unique<PyJointPositionControl>(self);
                 p->start();
                 return p;
             },
             py::keep_alive<0, 1>())
        .def("start_cartesian_pose_control",
             [](PyRobot& self, PyControllerMode mode) {
                 auto p = std::make_unique<PyCartesianControl>(self, mode);
                 p->start();
                 return p;
             },
             py::arg("control_type") = PyControllerMode::CartesianImpedance,
             py::keep_alive<0, 1>(),
             "Start Cartesian pose control mode.\n\n"
             "  control_type: ControllerMode.JointImpedance for rigid pose\n"
             "    tracking, or ControllerMode.CartesianImpedance for compliant\n"
             "    end-effector behavior (recommended for teleop).\n\n"
             "Returns a CartesianControl object whose RT thread runs in C++,\n"
             "decoupled from the Python GIL.  Use set_target_pose(T) at any\n"
             "rate; the RT thread interpolates to 1 kHz and applies\n"
             "franka::limitRate per cycle.");

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

    py::class_<PyCartesianControl>(m, "CartesianControl",
        "1 kHz Cartesian pose streaming controller.\n\n"
        "Wraps libfranka's start_cartesian_pose_control + an internal C++ RT\n"
        "thread.  Python feeds target poses at any rate via set_target_pose;\n"
        "the RT thread interpolates to 1 kHz, runs franka::limitRate, jump\n"
        "detection, and writeOnce, all decoupled from the Python GIL.")
        .def("set_target_pose", &PyCartesianControl::setTargetPose,
             py::arg("O_T_EE"),
             "Enqueue a 4×4 column-major pose target.  Non-blocking.")
        .def("set_target_pose_with_elbow",
             &PyCartesianControl::setTargetPoseWithElbow,
             py::arg("O_T_EE"), py::arg("elbow"),
             "Enqueue a target pose plus an explicit elbow [J3 rad, J4 sign].")
        .def("get_state", &PyCartesianControl::getState,
             "Read the latest snapshot of robot state captured by the RT thread.")
        .def("move_to_pose", &PyCartesianControl::moveToPose,
             py::arg("O_T_EE"), py::arg("duration_sec") = 0.0,
             "Run a min-jerk trajectory from the current pose to O_T_EE in\n"
             "the RT thread.  Non-blocking; use is_moving() to poll.")
        .def("is_moving", &PyCartesianControl::isMoving)
        .def("cancel_move", &PyCartesianControl::cancelMove)
        .def("trigger_estop", &PyCartesianControl::triggerEstop)
        .def("stop", &PyCartesianControl::stop, "Idempotent.")
        .def("is_running", &PyCartesianControl::isRunning)
        .def("__enter__", [](PyCartesianControl& s) -> PyCartesianControl& { return s; })
        .def("__exit__", [](PyCartesianControl& s, py::object, py::object, py::object) {
            s.stop();
        });
}
