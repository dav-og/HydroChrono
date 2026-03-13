/*********************************************************************
 * @file  linear_directional_wave_field.cpp
 * @brief Implementation of LinearDirectionalWaveField.
 *********************************************************************/

#include <hydroc/waves/linear_directional_wave_field.h>
#include <hydroc/waves/linear_wave_kinematics.h>
#include <hydroc/math_constants.h>
#include <hydroc/logging.h>
#include "wave_utilities.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace {

// Numerical guard: sinh(kh) below this threshold triggers the shallow-water
// asymptotic form to avoid division-by-near-zero.
constexpr double kShallowWaterKhThreshold = 1e-8;

/// Compute the depth attenuation factors for horizontal and vertical
/// particle motion at elevation z in water of given depth.
///
/// Returns {H_cosh, H_sinh} where:
///   horizontal factor = omega * A * H_cosh * cos(phase)
///   vertical   factor = omega * A * H_sinh * sin(phase)
struct DepthAttenuation {
    double h_cosh;
    double h_sinh;
};

DepthAttenuation ComputeDepthAttenuation(double k_mag, double z, double water_depth) {
    if (is_in_deep_water(k_mag, water_depth)) {
        double ekz = std::exp(k_mag * z);
        return {ekz, ekz};
    }
    const double kh = k_mag * water_depth;
    if (kh < kShallowWaterKhThreshold) {
        return {1.0 / kh, (z + water_depth) / water_depth};
    }
    const double denom = std::sinh(kh);
    return {std::cosh(k_mag * (z + water_depth)) / denom,
            std::sinh(k_mag * (z + water_depth)) / denom};
}

}  // namespace

LinearDirectionalWaveField::LinearDirectionalWaveField(
    std::vector<WaveComponent> components,
    double depth)
    : components_(std::move(components)) {
    water_depth_ = depth;

    if (components_.size() == 1) {
        mode_ = WaveMode::regular;
    } else {
        mode_ = WaveMode::irregular;
    }

    PrecomputeArrays();
}

void LinearDirectionalWaveField::Initialize() {
    // Wavenumbers are already computed by ComponentSampler.
    // H5 data is provided separately via AddH5Data().
}

void LinearDirectionalWaveField::PrecomputeArrays() {
    const Eigen::Index n = static_cast<Eigen::Index>(components_.size());
    amplitudes_.resize(n);
    kx_.resize(n);
    ky_.resize(n);
    omegas_.resize(n);
    phases_.resize(n);
    cos_dirs_.resize(n);
    sin_dirs_.resize(n);
    wavenumbers_.resize(n);

    for (Eigen::Index i = 0; i < n; ++i) {
        const auto& c = components_[i];
        amplitudes_[i]  = c.amplitude;
        kx_[i]          = c.k * std::cos(c.direction);
        ky_[i]          = c.k * std::sin(c.direction);
        omegas_[i]      = c.omega;
        phases_[i]      = c.phase;
        cos_dirs_[i]    = std::cos(c.direction);
        sin_dirs_[i]    = std::sin(c.direction);
        wavenumbers_[i] = c.k;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics: elevation, velocity, acceleration
// ─────────────────────────────────────────────────────────────────────────────

double LinearDirectionalWaveField::GetElevation(const Eigen::Vector3d& position, double time) const {
    const double x = position.x();
    const double y = position.y();

    // Vectorized: phase_i = kx_i*x + ky_i*y - omega_i*t + phi_i
    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    return (amplitudes_.array() * phase_args.cos()).sum();
}

Eigen::Vector2d LinearDirectionalWaveField::GetElevationGradientXY(
    const Eigen::Vector3d& position, double time) const {
    const double x = position.x();
    const double y = position.y();

    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd neg_a_sin = -(amplitudes_.array() * phase_args.sin());

    double deta_dx = (neg_a_sin * kx_.array()).sum();
    double deta_dy = (neg_a_sin * ky_.array()).sum();

    return Eigen::Vector2d(deta_dx, deta_dy);
}

double LinearDirectionalWaveField::GetElevationForVisualization(
    const Eigen::Vector3d& position, double time, int max_components) const {
    const Eigen::Index n_total = amplitudes_.size();
    const Eigen::Index n = (max_components <= 0 || max_components >= n_total)
                           ? n_total
                           : static_cast<Eigen::Index>(max_components);

    const double x = position.x();
    const double y = position.y();

    const Eigen::ArrayXd phase_args = kx_.head(n).array() * x
                                    + ky_.head(n).array() * y
                                    - omegas_.head(n).array() * time
                                    + phases_.head(n).array();
    return (amplitudes_.head(n).array() * phase_args.cos()).sum();
}

Eigen::Vector3d LinearDirectionalWaveField::GetVelocity(
    const Eigen::Vector3d& position, double time, double elevation) const {
    auto pos = wave_stretching_
               ? GetWheelerStretchedPosition(position, elevation, water_depth_, mwl_)
               : position;

    const double x = pos.x();
    const double y = pos.y();
    const double z = pos.z() - mwl_;

    // Vectorized phase computation.
    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd cos_phase = phase_args.cos();
    const Eigen::ArrayXd sin_phase = phase_args.sin();

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    for (Eigen::Index i = 0; i < amplitudes_.size(); ++i) {
        const double k_mag = std::abs(wavenumbers_[i]);
        if (k_mag == 0.0 || omegas_[i] == 0.0 || amplitudes_[i] == 0.0) continue;

        const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(k_mag, z, water_depth_);

        const double u_horiz = omegas_[i] * amplitudes_[i] * H_cosh * cos_phase[i];
        const double u_vert  = omegas_[i] * amplitudes_[i] * H_sinh * sin_phase[i];

        velocity[0] += u_horiz * cos_dirs_[i];
        velocity[1] += u_horiz * sin_dirs_[i];
        velocity[2] += u_vert;
    }

    return velocity;
}

Eigen::Vector3d LinearDirectionalWaveField::GetAcceleration(
    const Eigen::Vector3d& position, double time, double elevation) const {
    auto pos = wave_stretching_
               ? GetWheelerStretchedPosition(position, elevation, water_depth_, mwl_)
               : position;

    const double x = pos.x();
    const double y = pos.y();
    const double z = pos.z() - mwl_;

    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd sin_phase = phase_args.sin();
    const Eigen::ArrayXd cos_phase = phase_args.cos();

    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    for (Eigen::Index i = 0; i < amplitudes_.size(); ++i) {
        const double k_mag = std::abs(wavenumbers_[i]);
        if (k_mag == 0.0 || omegas_[i] == 0.0 || amplitudes_[i] == 0.0) continue;

        const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(k_mag, z, water_depth_);

        const double w2 = omegas_[i] * omegas_[i];
        const double a_horiz =  w2 * amplitudes_[i] * H_cosh * sin_phase[i];
        const double a_vert  = -w2 * amplitudes_[i] * H_sinh * cos_phase[i];

        acceleration[0] += a_horiz * cos_dirs_[i];
        acceleration[1] += a_horiz * sin_dirs_[i];
        acceleration[2] += a_vert;
    }

    return acceleration;
}

// ─────────────────────────────────────────────────────────────────────────────
// Excitation forces
// ─────────────────────────────────────────────────────────────────────────────

void LinearDirectionalWaveField::AddH5Data(
    const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
    const HydroData::SimulationParameters& sim_data,
    const Eigen::VectorXd& h5_wave_directions) {

    const double prev_depth = water_depth_;
    const double prev_g     = g_;
    water_depth_ = sim_data.water_depth;
    g_ = sim_data.g;

    const bool depth_changed = (std::abs(water_depth_ - prev_depth) > 1e-12)
                            || (std::abs(g_ - prev_g) > 1e-12);
    if (depth_changed) {
        for (auto& c : components_) {
            c.k = ComputeWaveNumber(c.omega, water_depth_, g_);
        }
        PrecomputeArrays();
    }

    if (!reg_h5_data.empty() && reg_h5_data[0].freq_list.size() > 0) {
        PrecomputeExcitationTransfer(reg_h5_data,
                                     reg_h5_data[0].freq_list,
                                     h5_wave_directions);
        has_h5_data_ = true;
    }
}

void LinearDirectionalWaveField::PrecomputeExcitationTransfer(
    const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
    const Eigen::VectorXd& h5_freq_list,
    const Eigen::VectorXd& h5_wave_directions) {

    const int n_bodies = static_cast<int>(reg_h5_data.size());
    const Eigen::Index n_comp = static_cast<Eigen::Index>(components_.size());
    const Eigen::Index n_h5_freq = h5_freq_list.size();
    const Eigen::Index n_h5_dir = h5_wave_directions.size();

    if (n_h5_dir <= 1 && n_comp > 1) {
        std::set<double> unique_dirs;
        for (const auto& c : components_) unique_dirs.insert(c.direction);
        if (unique_dirs.size() > 1) {
            LOG_WARNING("H5 data contains only " << n_h5_dir
                << " wave heading(s) but the sea state has "
                << unique_dirs.size() << " distinct component directions. "
                << "Excitation forces will use a single-heading approximation.");
        }
    }

    // Frequency grid parameters (use actual bounds, not assumed spacing).
    const double omega_min_h5 = (n_h5_freq > 0) ? h5_freq_list[0] : 0.0;
    const double omega_max_h5 = (n_h5_freq > 0) ? h5_freq_list[n_h5_freq - 1] : 0.0;
    const double omega_range  = omega_max_h5 - omega_min_h5;

    excitation_re_.resize(n_bodies);
    excitation_im_.resize(n_bodies);

    for (int b = 0; b < n_bodies; ++b) {
        excitation_re_[b].resize(kDofsPerBody, n_comp);
        excitation_im_[b].resize(kDofsPerBody, n_comp);

        for (Eigen::Index ci = 0; ci < n_comp; ++ci) {
            // Frequency interpolation index using actual grid bounds.
            int f_lo = 0;
            int f_hi = 0;
            double f_frac = 0.0;
            if (n_h5_freq == 1) {
                // Single frequency: no interpolation, use index 0 directly.
                f_lo = 0;
                f_hi = 0;
                f_frac = 0.0;
            } else {
                double freq_idx = 0.0;
                if (omega_range > 0.0) {
                    freq_idx = (components_[ci].omega - omega_min_h5) / omega_range
                               * static_cast<double>(n_h5_freq - 1);
                }
                f_lo = static_cast<int>(std::floor(freq_idx));
                f_lo = std::clamp(f_lo, 0, static_cast<int>(n_h5_freq - 2));
                f_hi = f_lo + 1;
                f_frac = std::clamp(freq_idx - static_cast<double>(f_lo), 0.0, 1.0);
            }

            // Heading interpolation index.
            int h_lo = 0;
            int h_hi = 0;
            double h_frac = 0.0;

            if (n_h5_dir > 1) {
                constexpr double kTwoPi  = 2.0 * M_PI;
                constexpr double kDirEps = 1e-12;

                double comp_dir = std::fmod(components_[ci].direction, kTwoPi);
                if (comp_dir < 0.0) comp_dir += kTwoPi;

                const double* dir_begin = h5_wave_directions.data();
                const double* dir_end   = dir_begin + n_h5_dir;
                const double* it = std::lower_bound(dir_begin, dir_end, comp_dir);

                if (it == dir_begin || it == dir_end) {
                    // comp_dir is below the first heading or above the last:
                    // wrap around between the last and first headings.
                    h_lo = static_cast<int>(n_h5_dir - 1);
                    h_hi = 0;
                    double gap = (h5_wave_directions[0] + kTwoPi) - h5_wave_directions[h_lo];
                    if (gap > kDirEps) {
                        double dist = comp_dir - h5_wave_directions[h_lo];
                        if (dist < 0.0) dist += kTwoPi;
                        h_frac = dist / gap;
                        h_frac = std::clamp(h_frac, 0.0, 1.0);
                    }
                } else {
                    h_lo = static_cast<int>(it - dir_begin - 1);
                    h_hi = h_lo + 1;
                    const double dir_lo = h5_wave_directions[h_lo];
                    const double dir_hi = h5_wave_directions[h_hi];
                    if (std::abs(dir_hi - dir_lo) > kDirEps) {
                        h_frac = (comp_dir - dir_lo) / (dir_hi - dir_lo);
                        h_frac = std::clamp(h_frac, 0.0, 1.0);
                    }
                }
            }
            // else: single heading -> h_lo=h_hi=0, h_frac=0

            for (int dof = 0; dof < kDofsPerBody; ++dof) {
                // Bilinear interpolation in Cartesian (Re/Im) form.
                // Converting mag/phase -> Re/Im before interpolation avoids
                // catastrophic errors when the phase crosses the +/-pi boundary.
                auto interp_freq = [&](int h_idx) -> std::pair<double, double> {
                    double m0 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_lo);
                    double p0 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_lo);
                    double m1 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_hi);
                    double p1 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_hi);
                    double re0 = m0 * std::cos(p0), im0 = m0 * std::sin(p0);
                    double re1 = m1 * std::cos(p1), im1 = m1 * std::sin(p1);
                    double re  = re0 + f_frac * (re1 - re0);
                    double im  = im0 + f_frac * (im1 - im0);
                    return {re, im};
                };

                auto [re_lo, im_lo] = interp_freq(h_lo);
                if (h_lo == h_hi) {
                    excitation_re_[b](dof, ci) = re_lo;
                    excitation_im_[b](dof, ci) = im_lo;
                } else {
                    auto [re_hi, im_hi] = interp_freq(h_hi);
                    excitation_re_[b](dof, ci) = re_lo + h_frac * (re_hi - re_lo);
                    excitation_im_[b](dof, ci) = im_lo + h_frac * (im_hi - im_lo);
                }
            }
        }
    }
}

Eigen::VectorXd LinearDirectionalWaveField::GetForceAtTime(double t) const {
    unsigned int total_dofs = num_bodies_ * kDofsPerBody;
    Eigen::VectorXd f = Eigen::VectorXd::Zero(total_dofs);

    if (!has_h5_data_ || excitation_re_.empty()) {
        return f;
    }

    const Eigen::Index n_comp = static_cast<Eigen::Index>(components_.size());

    for (unsigned int b = 0; b < num_bodies_; ++b) {
        unsigned int b_offset = kDofsPerBody * b;
        for (int dof = 0; dof < kDofsPerBody; ++dof) {
            double force_dof = 0.0;
            for (Eigen::Index ci = 0; ci < n_comp; ++ci) {
                const double theta = phases_[ci] - omegas_[ci] * t;
                force_dof += amplitudes_[ci] *
                    (excitation_re_[b](dof, ci) * std::cos(theta)
                   - excitation_im_[b](dof, ci) * std::sin(theta));
            }
            f[b_offset + dof] = force_dof;
        }
    }

    // Cosine ramp: 0.5*(1 - cos(pi*t/T_ramp)), matching IrregularWaves convention.
    if (ramp_duration_ > 0.0 && t < ramp_duration_) {
        double ramp = (t <= 0.0) ? 0.0
                    : 0.5 * (1.0 - std::cos(M_PI * t / ramp_duration_));
        f *= ramp;
    }

    return f;
}
