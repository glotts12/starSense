#pragma once

#include "types.hpp"

namespace starSense {

struct ReferenceState {
    Quat qRef;
    Vec3 wRef;
};

class ReferenceProfile {
public:
    virtual ~ReferenceProfile() = default;

    // Baseline signature; we can later add more info if needed
    virtual ReferenceState computeReferenceState(double t, AttitudeState estimatedState) const = 0;
};

class ConstantReferenceProfile : public ReferenceProfile {
public:
    explicit ConstantReferenceProfile(const Quat& qRef0)
        : qRef0_(qRef0) {}

    ReferenceState computeReferenceState(double /*t*/, AttitudeState /*estimatedState*/) const override {
        return ReferenceState{qRef0_, Vec3{0.0, 0.0, 0.0}};
    }

private:
    const Quat qRef0_;
};

class SpinningReferenceProfile : public ReferenceProfile {
public:
    explicit SpinningReferenceProfile(const Vec3 wRef0) 
        : wRef0_(wRef0) {}

    ReferenceState computeReferenceState(double /*t*/, AttitudeState estimatedState) const override {
        return ReferenceState{estimatedState.q, wRef0_};
    }

private:
    const Vec3 wRef0_;
};

} // namespace starSense
