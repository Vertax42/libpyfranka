#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

std::atomic_bool g_stop{false};

void SignalHandler(int) {
  g_stop.store(true);
}

struct Args {
  std::string ip = "192.168.99.111";
  std::string mode = "velocity";
  std::string controller = "cartesian";
  double amp = 0.005;
  double freq = 0.1;
  double duration = 30.0;
  double ramp = 2.0;
  bool limit_rate = true;
  bool yes = false;
};

void Usage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " --ip <robot-ip> [options]\n\n"
      << "Official libfranka on-robot Cartesian sine diagnostic.\n"
      << "This bypasses franka_rt and uses robot.control(...) callbacks directly.\n\n"
      << "Options:\n"
      << "  --mode pose|velocity|both      Motion generator to test (default velocity)\n"
      << "  --controller cartesian|joint   Internal controller mode (default cartesian)\n"
      << "  --amp <m>                      Sine amplitude in metres (default 0.005)\n"
      << "  --freq <hz>                    Sine frequency in Hz (default 0.1)\n"
      << "  --duration <s>                 Run duration per mode (default 30)\n"
      << "  --ramp <s>                     Min-jerk ramp-in/out duration (default 2)\n"
      << "  --no-limit-rate                Pass limit_rate=false to robot.control\n"
      << "  --yes                          Do not prompt before motion\n";
}

double ParseDouble(const char* s, const char* name) {
  char* end = nullptr;
  double v = std::strtod(s, &end);
  if (end == s || *end != '\0') {
    throw std::invalid_argument(std::string("Invalid ") + name + ": " + s);
  }
  return v;
}

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument(std::string("Missing value for ") + name);
      return argv[++i];
    };
    if (a == "--help" || a == "-h") {
      Usage(argv[0]);
      std::exit(0);
    } else if (a == "--ip") {
      args.ip = need_value("--ip");
    } else if (a == "--mode") {
      args.mode = need_value("--mode");
    } else if (a == "--controller") {
      args.controller = need_value("--controller");
    } else if (a == "--amp") {
      args.amp = ParseDouble(need_value("--amp"), "--amp");
    } else if (a == "--freq") {
      args.freq = ParseDouble(need_value("--freq"), "--freq");
    } else if (a == "--duration") {
      args.duration = ParseDouble(need_value("--duration"), "--duration");
    } else if (a == "--ramp") {
      args.ramp = ParseDouble(need_value("--ramp"), "--ramp");
    } else if (a == "--no-limit-rate") {
      args.limit_rate = false;
    } else if (a == "--yes") {
      args.yes = true;
    } else {
      throw std::invalid_argument("Unknown argument: " + a);
    }
  }

  if (args.mode != "pose" && args.mode != "velocity" && args.mode != "both") {
    throw std::invalid_argument("--mode must be pose, velocity, or both");
  }
  if (args.controller != "cartesian" && args.controller != "joint") {
    throw std::invalid_argument("--controller must be cartesian or joint");
  }
  if (args.amp < 0.0 || args.freq < 0.0 || args.duration <= 0.0 || args.ramp < 0.0) {
    throw std::invalid_argument("amp/freq/ramp must be non-negative and duration must be positive");
  }
  return args;
}

franka::ControllerMode ControllerMode(const Args& args) {
  return args.controller == "cartesian"
      ? franka::ControllerMode::kCartesianImpedance
      : franka::ControllerMode::kJointImpedance;
}

void ApplyDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  robot.setJointImpedance({{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0}});
  robot.setCartesianImpedance({{3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0}});
}

void PrintState(const franka::RobotState& s) {
  std::cout << "  EE pos: ["
            << s.O_T_EE[12] << ", " << s.O_T_EE[13] << ", " << s.O_T_EE[14]
            << "] m\n";
  std::cout << "  q     : [";
  for (size_t i = 0; i < s.q.size(); ++i) {
    std::cout << (i ? ", " : "") << s.q[i];
  }
  std::cout << "] rad\n";
  std::cout << "  elbow : [" << s.elbow[0] << ", " << s.elbow[1] << "]\n";
}

double SmoothStep(double t, double ramp) {
  if (ramp <= 0.0 || t >= ramp) return 1.0;
  if (t <= 0.0) return 0.0;
  double s = t / ramp;
  return 10.0 * s * s * s - 15.0 * s * s * s * s + 6.0 * s * s * s * s * s;
}

double SmoothStepDerivative(double t, double ramp) {
  if (ramp <= 0.0 || t <= 0.0 || t >= ramp) return 0.0;
  double s = t / ramp;
  double ds = 30.0 * s * s - 60.0 * s * s * s + 30.0 * s * s * s * s;
  return ds / ramp;
}

double RampOutScale(double remaining, double ramp) {
  return SmoothStep(remaining, ramp);
}

void Confirm(const Args& args, const std::string& mode) {
  std::cout << "\nAbout to run OFFICIAL libfranka " << mode << " sine.\n"
            << "  ip        : " << args.ip << "\n"
            << "  controller: " << args.controller << " impedance\n"
            << "  amp/freq  : " << args.amp << " m / " << args.freq << " Hz\n"
            << "  duration  : " << args.duration << " s\n"
            << "  limitRate : " << (args.limit_rate ? "true" : "false") << "\n"
            << "Keep e-stop in hand.\n";
  if (!args.yes) {
    std::cout << "Press Enter to start, Ctrl+C to abort ... " << std::flush;
    std::string line;
    std::getline(std::cin, line);
  }
}

void RunPose(franka::Robot& robot, const Args& args) {
  Confirm(args, "CartesianPose callback");
  auto initial_state = robot.readOnce();
  auto initial_pose = initial_state.O_T_EE;

  double time = 0.0;
  uint64_t cycles = 0;
  auto callback = [&](const franka::RobotState&, franka::Duration period) -> franka::CartesianPose {
    time += period.toSec();
    ++cycles;

    double omega = 2.0 * M_PI * args.freq;
    double env = SmoothStep(time, args.ramp) * RampOutScale(args.duration - time, args.ramp);

    auto pose = initial_pose;
    pose[12] = initial_pose[12] + args.amp * env * std::sin(omega * time);

    if (g_stop.load() || time >= args.duration) {
      return franka::MotionFinished(franka::CartesianPose(pose));
    }
    return franka::CartesianPose(pose);
  };

  robot.control(callback, ControllerMode(args), args.limit_rate);
  std::cout << "  pose callback completed: cycles=" << cycles << " time=" << time << " s\n";
}

void RunVelocity(franka::Robot& robot, const Args& args) {
  Confirm(args, "CartesianVelocities callback");

  double time = 0.0;
  uint64_t cycles = 0;
  auto callback =
      [&](const franka::RobotState&, franka::Duration period) -> franka::CartesianVelocities {
    time += period.toSec();
    ++cycles;

    double omega = 2.0 * M_PI * args.freq;
    double env_in = SmoothStep(time, args.ramp);
    double denv_in = SmoothStepDerivative(time, args.ramp);
    double remaining = args.duration - time;
    double env_out = RampOutScale(remaining, args.ramp);
    double denv_out = -SmoothStepDerivative(remaining, args.ramp);
    double env = env_in * env_out;
    double denv = denv_in * env_out + env_in * denv_out;

    double vx = args.amp * (denv * std::sin(omega * time) +
                            env * omega * std::cos(omega * time));
    std::array<double, 6> v{{vx, 0.0, 0.0, 0.0, 0.0, 0.0}};

    if (g_stop.load() || time >= args.duration) {
      std::array<double, 6> zero{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      return franka::MotionFinished(franka::CartesianVelocities(zero));
    }
    return franka::CartesianVelocities(v);
  };

  robot.control(callback, ControllerMode(args), args.limit_rate);
  std::cout << "  velocity callback completed: cycles=" << cycles << " time=" << time << " s\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  try {
    Args args = ParseArgs(argc, argv);

    std::cout << "Connecting to Franka at " << args.ip << "...\n";
    franka::Robot robot(args.ip, franka::RealtimeConfig::kIgnore);
    ApplyDefaultBehavior(robot);

    std::cout << "Initial state:\n";
    PrintState(robot.readOnce());

    if (args.mode == "pose" || args.mode == "both") {
      RunPose(robot, args);
      if (args.mode == "both") {
        std::cout << "Waiting 1 s before next mode...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    if (args.mode == "velocity" || args.mode == "both") {
      RunVelocity(robot, args);
    }

    std::cout << "Done.\n";
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "\nlibfranka exception: " << e.what() << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "\nerror: " << e.what() << "\n";
    return 1;
  }
}
