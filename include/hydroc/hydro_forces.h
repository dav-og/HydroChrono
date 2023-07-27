#include <cstdio>
#include <filesystem>

#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChSolverPMINRES.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/timestepper/ChTimestepperHHT.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChForce.h>
#include <chrono/physics/ChLoad.h>
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChSystemSMC.h>

#include <chrono/fea/ChMeshFileLoader.h>

#include <hydroc/h5fileinfo.h>
#include <hydroc/wave_types.h>

#include "C:\\code\\MoorDyn_hc\\MoorDyn\\source\\MoorDyn2.h"

using namespace chrono;
using namespace chrono::fea;

/////@todo eventually add irregular waves mode
// enum class WaveMode {
//    /// @brief No waves
//    noWaveCIC = 0,
//    /// @brief Regular waves
//    regular = 1,
//    /// @brief Irregular waves
//    irregular = 2
//};

// =============================================================================
// old HydroInputs struct (reg wave details here)
// struct HydroInputs {
//    WaveMode mode;
//    HydroInputs();
//    double freq_index_des;
//    double regular_wave_amplitude;
//    double regular_wave_omega;
//    double wave_omega_delta;
//    std::vector<double> excitation_force_mag;
//    std::vector<double> excitation_force_phase;
//    HydroInputs(HydroInputs& old) = default;
//    HydroInputs& operator=(const HydroInputs& rhs) = default;
//};

// =============================================================================
class ForceFunc6d;

class TestHydro;

class ComponentFunc : public ChFunction {
  public:
    ComponentFunc();
    ComponentFunc(const ComponentFunc& old);
    ComponentFunc(ForceFunc6d* b, int i);
    virtual ComponentFunc* Clone() const override;
    virtual double Get_y(double x) const override;

  private:
    ForceFunc6d* base;
    int index;
};

// =============================================================================
// ForceFunc6d organizes the functional (time dependent) forces in each DoF (6 total) for a body
class ForceFunc6d {
  public:
    ForceFunc6d();
    ForceFunc6d(std::shared_ptr<ChBody> object, TestHydro* all_hydro_forces_user);
    ForceFunc6d(const ForceFunc6d& old);
    double coordinateFunc(int i);

  private:
    void SetForce();
    void SetTorque();
    void ApplyForceAndTorqueToBody();
    std::shared_ptr<ChBody> body;
    int b_num;
    ComponentFunc forces[6];
    std::shared_ptr<ComponentFunc> force_ptrs[6];
    std::shared_ptr<ChForce> chrono_force;
    std::shared_ptr<ChForce> chrono_torque;
    TestHydro* all_hydro_forces;
};

class ChLoadAddedMass;

class MoorDynDLLHandler {
  public:
    // Constructor and destructor
    MoorDynDLLHandler();
    ~MoorDynDLLHandler();

    // Other public methods
    void LoadMoorDyn(std::string dll_path);
    MoorDyn CreateMoorDyn(const char* infilename);
    int DofsMoorDyn(MoorDyn system, unsigned int* n);
    int InitMoorDyn(MoorDyn system, const double* x, const double* xd);
    int StepMoorDyn(MoorDyn system, const double* x, const double* xd, double* f, double* t, double* dt);
    void CloseMoorDyn();

    MoorDyn GetMoorDynSystem() { return moordyn_system; }
    void SetMoorDynSystem(MoorDyn system) { moordyn_system = system; }

  private:
    HMODULE hMod;
    MoorDyn (*MoorDynCreate)(const char* infilename);
    int (*MoorDynDofs)(MoorDyn system, unsigned int* n);
    int (*MoorDynInit)(MoorDyn system, const double* x, const double* xd);
    int (*MoorDynStep)(MoorDyn system, const double* x, const double* xd, double* f, double* t, double* dt);
    int (*MoorDynClose)(MoorDyn system);
    MoorDyn moordyn_system;
};

class TestHydro {
  public:
    bool printed = false;
    TestHydro()  = delete;
    TestHydro(std::vector<std::shared_ptr<ChBody>> user_bodies,
              std::string h5_file_name,
              std::shared_ptr<WaveBase> waves);
    TestHydro(std::vector<std::shared_ptr<ChBody>> user_bodies, std::string h5_file_name)
        : TestHydro(user_bodies, h5_file_name, std::static_pointer_cast<WaveBase>(std::make_shared<NoWave>())) {}
    TestHydro(const TestHydro& old) = delete;
    TestHydro operator=(const TestHydro& rhs) = delete;
    void AddWaves(std::shared_ptr<WaveBase> waves);
    void WaveSetUp();

    /**
     * @brief Adds Mooring lines to single body.
     * 
     * This function initializes any variables needed for moordyn.
     *
     * @param lines string location of input file lines.txt
     * @param dll_loc string location of moordyn dll
     * @param bod_name string name of body to attach moorings to, must match body name from h5 file
     */
    //void AddMoorings(std::string lines, std::string dll_loc, std::string bod_name);
    void AddMoorDyn(std::string moorDynInputPath,
                    std::string moorDynDllPath,
                    std::vector<std::string> bodyNames);
    /**
     * @brief Computes the mooring force for all bodies with moorings attached.
     * @return 6N dimensional mooring force
    */
    Eigen::VectorXd ComputeForceMoorDyn();
    std::vector<double> ComputeForceHydrostatics();
    std::vector<double> ComputeForceRadiationDampingConv();
    Eigen::VectorXd ComputeForceWaves();
    // std::vector<double> ComputeForceExcitationRegularFreq();
    // double ExcitationConvolution(int body,
    //                             int dof,
    //                             double t,
    //                             const std::vector<double>& eta,
    //                             // const std::vector<double>& excitation_irf,
    //                             const Eigen::VectorXd& t_irf_new,
    //                             double sim_dt);
    // std::vector<double> ComputeForceExcitation();
    double GetRIRFval(int row, int col, int st);
    double coordinateFunc(int b, int i);
    bool convTrapz;
    Eigen::VectorXd t_irf;

    void EndSimulation();

  private:
    std::vector<std::shared_ptr<ChBody>> bodies;
    int num_bodies;
    bool moordyn_on = false;

    /**
     * @brief list of pairs of the 0 indexed body to attach moorings to from line.txt file given by string.
    */
    std::vector<std::pair<int, std::string>> moordyn_input;
    HydroData file_info;
    std::vector<ForceFunc6d> force_per_body;
    double sumVelHistoryAndRIRF;
    // HydroInputs hydro_inputs;
    std::shared_ptr<WaveBase> user_waves;
    std::vector<double> force_hydrostatic;
    std::vector<double> force_radiation_damping;
    Eigen::VectorXd force_waves;
    Eigen::VectorXd force_mooring;
    // std::vector<double> force_excitation;
    std::vector<double> total_force;
    std::vector<double> equilibrium;
    std::vector<double> cb_minus_cg;
    double rirf_timestep;
    double getVelHistoryVal(int step, int c) const;
    double setVelHistory(double val, int step, int b_num, int index);

    // double freq_index_des;
    // int freq_index_floor;
    // double freq_interp_val;
    std::vector<double> velocity_history;  // use helper function to access vel_history elements correctly
    double prev_time;
    Eigen::VectorXd rirf_time_vector;  // (should be the same for each body?)
    int offset_rirf;
    std::shared_ptr<ChLoadContainer> my_loadcontainer;
    std::shared_ptr<ChLoadAddedMass> my_loadbodyinertia;
    std::vector<std::unique_ptr<MoorDynDLLHandler>> dllHandlers;
};