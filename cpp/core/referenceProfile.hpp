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
    virtual ReferenceState computeReferenceState(double t) const = 0;
};

class ConstantReferenceProfile : public ReferenceProfile {
public:
    explicit ConstantReferenceProfile(const Quat& qRef0)
        : qRef0_(qRef0) {}

    ReferenceState computeReferenceState(double /*t*/) const override {
        return ReferenceState{qRef0_, Vec3{0.0, 0.0, 0.0}};
    }

private:
    const Quat qRef0_;
};

} // namespace starSense
