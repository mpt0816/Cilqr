#pragma once

namespace planning {

struct PidParam {
  double kp = 0.0;
	double ki = 0.0;
	double kd = 0.0;
};

class Pid {
 public:
  Pid() = default;
};

} // namespace planning