/*********************************************************************
 * @file  linear_damping_component.cpp
 * @brief Implementation of per-DOF linear damping force component.
 *********************************************************************/

#include "linear_damping_component.h"

#include <stdexcept>

namespace hydrochrono::hydro {

LinearDampingComponent::LinearDampingComponent(
    const std::vector<std::array<double, 6>>& per_body)
    : coefficients_(per_body) {
    if (coefficients_.empty()) {
        throw std::invalid_argument(
            "LinearDampingComponent: per_body coefficients must not be empty");
    }
}

void LinearDampingComponent::Compute(const SystemState& state,
                                     double /*time*/,
                                     BodyForces& inout_forces) {
    const int n = static_cast<int>(coefficients_.size());
    for (int b = 0; b < n; ++b) {
        const auto& body = state.bodies[b];
        const auto& B = coefficients_[b];

        inout_forces[b][0] -= B[0] * body.linear_velocity.x();
        inout_forces[b][1] -= B[1] * body.linear_velocity.y();
        inout_forces[b][2] -= B[2] * body.linear_velocity.z();
        inout_forces[b][3] -= B[3] * body.angular_velocity.x();
        inout_forces[b][4] -= B[4] * body.angular_velocity.y();
        inout_forces[b][5] -= B[5] * body.angular_velocity.z();
    }
}

}  // namespace hydrochrono::hydro
