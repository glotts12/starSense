#pragma once

#include "types.hpp"

namespace starSense {

// Abstract sensor interface
class Sensor {
public:
    virtual ~Sensor() = default;

    // Measure attitude from the true state.
    //
    // t          : current simulation time [s]
    // trueState  : true attitude state (q, w)
    //
    // Returns:
    //   measured quaternion (body wrt inertial)
    virtual Quat measureAttitude(
        double t,
        const AttitudeState &trueState
    ) const = 0;
};


// Release 1: Ideal attitude sensor
// No noise, no bias, no latency: just returns the true quaternion.
class IdealAttitudeSensor : public Sensor {
public:
    Quat measureAttitude(
        double t,
        const AttitudeState &trueState
    ) const override;
};

} // namespace starSense
