/*********************************************************************
 * @file  linear_damping_component.h
 * @brief Optional per-DOF linear damping: F_i = -B_i * v_i.
 *
 * Supplements potential-flow radiation damping with user-specified
 * viscous-like damping coefficients.  Particularly important for
 * DOFs where radiation damping is negligible (e.g. roll of slender
 * cylinders).
 *********************************************************************/

#ifndef HYDRO_FORCE_COMPONENTS_LINEAR_DAMPING_COMPONENT_H
#define HYDRO_FORCE_COMPONENTS_LINEAR_DAMPING_COMPONENT_H

#include <hydroc/core/force_component.h>
#include <array>
#include <vector>

namespace hydrochrono::hydro {

class LinearDampingComponent : public IHydroForceComponent {
  public:
    /// @param per_body  Per-body damping coefficients [surge, sway, heave, roll, pitch, yaw].
    ///                  Units: N·s/m (translational) or N·m·s/rad (rotational).
    explicit LinearDampingComponent(const std::vector<std::array<double, 6>>& per_body);

    HydroComponentType Type() const override { return HydroComponentType::LinearDamping; }

    void Compute(const SystemState& state,
                 double time,
                 BodyForces& inout_forces) override;

  private:
    std::vector<std::array<double, 6>> coefficients_;
    static constexpr int kDofPerBody = 6;
};

}  // namespace hydrochrono::hydro

#endif  // HYDRO_FORCE_COMPONENTS_LINEAR_DAMPING_COMPONENT_H
