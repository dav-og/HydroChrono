/*********************************************************************
 * @file  component_sampler.h
 * @brief Converts a SeaStateDefinition into a list of WaveComponents.
 *
 * The sampler is the bridge between declarative sea-state descriptions
 * and the component list that drives LinearDirectionalWaveField.
 *********************************************************************/

#ifndef HYDROC_WAVES_COMPONENT_SAMPLER_H
#define HYDROC_WAVES_COMPONENT_SAMPLER_H

#include <hydroc/waves/wave_component.h>
#include <random>
#include <vector>

class ComponentSampler {
  public:
    /// Build a list of wave components from a declarative sea-state definition.
    ///
    /// The returned vector is the canonical representation of the sea state
    /// and can be passed directly to LinearDirectionalWaveField.
    ///
    /// Handles all sea-state types:
    ///   - Regular:               1 component
    ///   - Long-crested irregular: n_omega components (n_theta == 1)
    ///   - Directional irregular:  n_omega * n_theta components per partition
    ///   - Bimodal:               union of components from all partitions
    [[nodiscard]] static std::vector<WaveComponent> Build(const SeaStateDefinition& def);

  private:
    /// Sample components for a single spectral partition.
    static std::vector<WaveComponent> SamplePartition(
        const SeaStatePartition& partition,
        int n_omega,
        int n_theta,
        double omega_min,
        double omega_max,
        double depth,
        double g,
        std::mt19937& rng);

    /// Remove components whose amplitude is negligibly small.
    static void PruneComponents(std::vector<WaveComponent>& components,
                                double threshold_fraction = 1e-6);
};

#endif  // HYDROC_WAVES_COMPONENT_SAMPLER_H
