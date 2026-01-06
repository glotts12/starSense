#pragma once

#include "types.hpp"
#include "util.hpp"
#include "referenceProfile.hpp"

namespace starSense {

// Abstract controller interface
class Controller {
public:
    virtual ~Controller() = default;

    // Compute commanded body-frame torque [N·m]
    //  t              : current simulation time [s]
    //  estimatedState : estimated attitude state (q, w)
    //  reference      : desired attitude profile (qRef, wRef)
    virtual Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceProfile &reference
    ) const = 0;
};


// Release 1: Zero controller
class ZeroController : public Controller {
public:
    Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceProfile &reference
    ) const override;
};


// PD controller placeholder (rate-only for now)
class PDController : public Controller {
public:
    PDController(double kpAtt, double kdRate);

    Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceProfile &reference
    ) const override;

private:
    double kpAtt_;   // attitude gain (unused for now)
    double kdRate_;  // rate damping gain
};

} // namespace starSense
