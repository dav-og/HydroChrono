/*********************************************************************
 * @file  linear_directional_wave_field.h
 * @brief Unified wave field: evaluates an arbitrary set of linear wave
 *        components via the standard WaveBase interface.
 *
 * This single class handles regular, long-crested irregular, directional
 * irregular, and bimodal sea states -- all as special cases of the same
 * component list.
 *********************************************************************/

#ifndef HYDROC_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H
#define HYDROC_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H

#include <hydroc/waves/wave_base.h>
#include <hydroc/waves/wave_component.h>
#include <hydroc/io/h5_reader.h>

#include <Eigen/Core>
#include <vector>

class LinearDirectionalWaveField : public WaveBase {
  public:
    /// Construct from a sampled component list.
    /// @param components  Sampled wave components (from ComponentSampler::Build)
    /// @param depth       Water depth [m] (0 or inf = deep water)
    explicit LinearDirectionalWaveField(std::vector<WaveComponent> components, double depth);

    void Initialize() override;
    Eigen::VectorXd GetForceAtTime(double t) const override;
    WaveMode GetWaveMode() const override { return mode_; }

    double GetElevation(const Eigen::Vector3d& position, double time) const override;
    Eigen::Vector3d GetVelocity(const Eigen::Vector3d& position, double time, double elevation) const override;
    Eigen::Vector3d GetAcceleration(const Eigen::Vector3d& position, double time, double elevation) const override;

    /// Surface slope (d_eta/dx, d_eta/dy) at a given position and time.
    Eigen::Vector2d GetElevationGradientXY(const Eigen::Vector3d& position, double time) const override;

    /// Elevation using only the first max_components components (for faster visualization).
    double GetElevationForVisualization(const Eigen::Vector3d& position, double time, int max_components) const;

    /// Provide H5 excitation data for force computation.
    /// Must be called before GetForceAtTime() is used.
    void AddH5Data(const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
                   const HydroData::SimulationParameters& sim_data,
                   const Eigen::VectorXd& h5_wave_directions);

    /// Access the raw component list (for logging, plotting, diagnostics).
    [[nodiscard]] const std::vector<WaveComponent>& GetComponents() const { return components_; }

    /// Set the excitation ramp duration [s]. 0 = no ramp (instantaneous full force).
    void SetRampDuration(double seconds) { ramp_duration_ = seconds; }

  private:
    WaveMode mode_ = WaveMode::irregular;
    double ramp_duration_ = 0.0;

    std::vector<WaveComponent> components_;

    // Pre-computed SIMD-friendly arrays (one element per component).
    Eigen::VectorXd amplitudes_;    ///< A_i
    Eigen::VectorXd kx_;            ///< k_i * cos(theta_i)
    Eigen::VectorXd ky_;            ///< k_i * sin(theta_i)
    Eigen::VectorXd omegas_;        ///< omega_i
    Eigen::VectorXd phases_;        ///< phi_i
    Eigen::VectorXd cos_dirs_;      ///< cos(theta_i)
    Eigen::VectorXd sin_dirs_;      ///< sin(theta_i)
    Eigen::VectorXd wavenumbers_;   ///< k_i

    /// Pre-compute vectorized arrays from the component list.
    void PrecomputeArrays();

    // --- Excitation force data (Cartesian form) ---
    // Per-component real/imaginary parts of the excitation transfer function,
    // interpolated from H5 mag/phase data.  Stored in Cartesian form to avoid
    // phase-wrapping artefacts during interpolation.
    // excitation_re_[body](dof, component_index)  = Re(H)
    // excitation_im_[body](dof, component_index)  = Im(H)
    std::vector<Eigen::MatrixXd> excitation_re_;
    std::vector<Eigen::MatrixXd> excitation_im_;
    bool has_h5_data_ = false;

    /// Interpolate excitation data from H5 tensors for each wave component.
    void PrecomputeExcitationTransfer(
        const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
        const Eigen::VectorXd& h5_freq_list,
        const Eigen::VectorXd& h5_wave_directions);
};

#endif  // HYDROC_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H
