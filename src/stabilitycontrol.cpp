#include "stabilitycontrol.hpp"

StabilityControl::StabilityControl()
{
  controllers.x = PID({1,0,0});
  controllers.y = PID({1,0,0});
  controllers.z = PID({1,0,0});
  controllers.h = PID({1,0,0});
}

StabilityControl::StabilityControl(Gains4 gains)
{
  controllers.x = PID(gains.x);
  controllers.y = PID(gains.y);
  controllers.z = PID(gains.z);
  controllers.h = PID(gains.h);
}

vec4f StabilityControl::getCommand(vec4f orientation,
                                   vec4f instruction,
                                   uint16_t delta_time)
{
  vec4f command;

  command.x = controllers.x.getCorrection(instruction.x,
                                          orientation.x,
                                          delta_time);

  command.y = controllers.y.getCorrection(instruction.y,
                                          orientation.y,
                                          delta_time);

  command.z = controllers.z.getCorrection(instruction.z,
                                          orientation.z,
                                          delta_time);

  command.h = controllers.h.getCorrection(instruction.h,
                                          orientation.h,
                                          delta_time);

  return command;
};

void StabilityControl::reset()
{
  controllers.x.reset();
  controllers.y.reset();
  controllers.z.reset();
  controllers.h.reset();
}
