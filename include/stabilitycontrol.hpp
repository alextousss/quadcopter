#ifndef DEF_STABILITY
#define DEF_STABILITY

#include "datastructs.hpp"
#include "PID.hpp"

struct Controllers
{
  PID x;
  PID y;
  PID z;
  PID h;
};


class StabilityControl
{
public:
  StabilityControl();
  StabilityControl(Gains4 gains);
  void reset();
  void setGainX(gain3f gain) { controllers.x.setGains(gain); }
  void setGainY(gain3f gain) { controllers.y.setGains(gain); }
  void setGainZ(gain3f gain) { controllers.z.setGains(gain); }
  void setGainH(gain3f gain) { controllers.h.setGains(gain); }

  vec4f getProportionalCorrection()
  {
    return {controllers.x.getProportionalCorrection(),
            controllers.y.getProportionalCorrection(),
            controllers.z.getProportionalCorrection(),
            controllers.h.getProportionalCorrection()};
  }
  vec4f getDerivateCorrection()
  {
    return {controllers.x.getDerivateCorrection(),
            controllers.y.getDerivateCorrection(),
            controllers.z.getDerivateCorrection(),
            controllers.h.getDerivateCorrection()};
  }
  vec4f getIntegralCorrection()
  {
    return {controllers.x.getIntegralCorrection(),
            controllers.y.getIntegralCorrection(),
            controllers.z.getIntegralCorrection(),
            controllers.h.getIntegralCorrection()};
  }
vec4f getCommand(vec4f orientation,
                   vec4f instruction,
                   uint16_t delta_time);
private:
  Controllers controllers;
};

#endif
