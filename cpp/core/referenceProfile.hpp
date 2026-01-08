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
    ConstantReferenceProfile(const Quat& qRef0, const Vec3& wRef0)
        : qRef0_(qRef0), wRef0_(wRef0) {}

    ReferenceState computeReferenceState(double /*t*/) const override {
        return ReferenceState{qRef0_, wRef0_};
    }

private:
    const Quat qRef0_;
    const Vec3 wRef0_;
};

} // namespace starSense
