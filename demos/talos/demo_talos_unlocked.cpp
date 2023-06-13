#include <hydroc/helper.h>
#include <hydroc/hydro_forces.h>

#ifdef HYDROCHRONO_HAVE_IRRLICHT
    #include <chrono_irrlicht/ChIrrMeshTools.h>
    #include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
using namespace chrono::irrlicht;
#endif

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChLinkMate.h>

#include <chrono>   // std::chrono::high_resolution_clock::now
#include <iomanip>  // std::setprecision
#include <vector>   // std::vector<double>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;

#ifdef HYDROCHRONO_HAVE_IRRLICHT
// Define a class to manage user inputs via the GUI (i.e. play/pause button)
class MyActionReceiver : public IEventReceiver {
  public:
    MyActionReceiver(ChVisualSystemIrrlicht* vsys, bool& buttonPressed) : pressed(buttonPressed) {
        // store pointer application
        vis = vsys;

        // ..add a GUI button to control pause/play
        pauseButton = vis->GetGUIEnvironment()->addButton(rect<s32>(510, 20, 650, 35));
        buttonText  = vis->GetGUIEnvironment()->addStaticText(L"Paused", rect<s32>(560, 20, 600, 35), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user clicked button
        if (event.EventType == EET_GUI_EVENT) {
            switch (event.GUIEvent.EventType) {
                case EGET_BUTTON_CLICKED:
                    pressed = !pressed;
                    if (pressed) {
                        buttonText->setText(L"Playing");
                    } else {
                        buttonText->setText(L"Paused");
                    }
                    return pressed;
                    break;
                default:
                    break;
            }
        }
        return false;
    }

  private:
    ChVisualSystemIrrlicht* vis;
    IGUIButton* pauseButton;
    IGUIStaticText* buttonText;

    bool& pressed;
};
#endif

int main(int argc, char* argv[]) {
    // auto start = std::chrono::high_resolution_clock::now();
    GetLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";

    if (hydroc::setInitialEnvironment(argc, argv) != 0) {
        return 1;
    }

    std::filesystem::path DATADIR(hydroc::getDataDir());

    auto hull_mesh_fname = (DATADIR / "talos" / "geometry" / "TALOS_G1_half.obj").lexically_normal().generic_string();
    auto h5_fname        = (DATADIR / "talos" / "hydroData" / "talos.h5").lexically_normal().generic_string();

    // system/solver settings
    ChSystemSMC system;
    system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
    double timestep = (20.0 / 1999);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::GMRES);
    system.SetSolverMaxIterations(300);  // the higher, the easier to keep the constraints satisfied.
    system.SetStep(timestep);
    ChRealtimeStepTimer realtime_timer;
    double simulation_duration = 1000.0;

    // some io/viz options
    bool visualization_on = false;
    bool profiling_on     = true;
    bool save_data_on     = true;
    std::vector<double> time_vector;
    std::vector<double> hull_surge;
    std::vector<double> hull_heave;
    std::vector<double> hull_pitch;
    // Create storage for power absorbed by each damper and total power absorbed
    std::vector<std::vector<double>> power_absorbed_by_spring(6);
    std::vector<double> total_power_absorbed;

    // set up hull from a mesh
    std::cout << "Attempting to open mesh file: " << hull_mesh_fname << std::endl;
    std::shared_ptr<ChBody> hull = chrono_types::make_shared<ChBodyEasyMesh>(  //
        hull_mesh_fname,
        0,      // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // collisions
    );

    // Create a visualization material
    auto yellow = chrono_types::make_shared<ChVisualMaterial>();
    yellow->SetDiffuseColor(ChColor(0.255f, 0.255f, 0.0f));
    yellow->SetOpacity(0.1);
    hull->GetVisualShape(0)->SetMaterial(0, yellow);
    system.Add(hull);
    hull->SetNameString("body1");

    // define the hull's initial conditions
    auto cg = ChVector<>(0.0, 0.0, 0.0);
    // offset used for heave/surge decay test
    auto offset = ChVector<>(0.0, 0.0, 0.0);
    hull->SetPos(cg + offset);
    // Use for pitch decay test
    double ang_rad = 0.0;  //-3.95 * CH_C_PI / 180.0;
    hull->SetRot(Q_from_AngAxis(ang_rad, VECT_Y));

    double total_mass = 3747762.0;

    hull->SetMass(total_mass * 0.8);
    hull->SetInertiaXX(ChVector<>(2.376e8, 2.376e8, 2.376e8));

    // internal reaction mass
    auto irm = chrono_types::make_shared<ChBodyEasySphere>(4.0, 1000.0, true, false);
    system.Add(irm);
    // mrigidBody->SetRot(randrot);
    irm->SetMass(total_mass * 0.2);
    irm->SetPos(ChVector<>(0.0, 0.0, -5.7));

    std::vector<double> stiffness_values = {1.0E+06, 5.0E+06, 10.0E+06, 15.0E+06, 20.0E+06};

    for (double current_stiffness : stiffness_values) {
        // PTO parameters
        double pto_length    = 5.0;
        double pto_stiffness = current_stiffness;
        double pto_damping   = 1.0E+05;
        double pto_radius    = 0.75;
        int pto_resolution   = 80;
        int pto_spring_turns = 10;

        // spring 1 between hull and irm
        auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
        spring_1->Initialize(hull, irm, true, ChVector<>(5.000, 0.000, -8.660 - 5.7), ChVector<>(2.500, 0.000, -4.330));
        spring_1->SetRestLength(pto_length);
        spring_1->SetSpringCoefficient(pto_stiffness);
        spring_1->SetDampingCoefficient(pto_damping);
        spring_1->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_1);

        // spring 2 between hull and irm
        auto spring_2 = chrono_types::make_shared<ChLinkTSDA>();
        spring_2->Initialize(hull, irm, true, ChVector<>(-2.500, 4.330, -8.660 - 5.7),
                             ChVector<>(-1.250, 2.165, -4.330));
        spring_2->SetRestLength(pto_length);
        spring_2->SetSpringCoefficient(pto_stiffness);
        spring_2->SetDampingCoefficient(pto_damping);
        spring_2->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_2);

        // spring 3 between hull and irm
        auto spring_3 = chrono_types::make_shared<ChLinkTSDA>();
        spring_3->Initialize(hull, irm, true, ChVector<>(-2.500, -4.330, -8.660 - 5.7),
                             ChVector<>(-1.250, -2.165, -4.330));
        spring_3->SetRestLength(pto_length);
        spring_3->SetSpringCoefficient(pto_stiffness);
        spring_3->SetDampingCoefficient(pto_damping);
        spring_3->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_3);

        // spring 4 between hull and irm
        auto spring_4 = chrono_types::make_shared<ChLinkTSDA>();
        spring_4->Initialize(hull, irm, true, ChVector<>(5.000, 0.000, 8.660 - 5.7), ChVector<>(2.500, 0.000, 4.330));
        spring_4->SetRestLength(pto_length);
        spring_4->SetSpringCoefficient(pto_stiffness);
        spring_4->SetDampingCoefficient(pto_damping);
        spring_4->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_4);

        // spring 5 between hull and irm
        auto spring_5 = chrono_types::make_shared<ChLinkTSDA>();
        spring_5->Initialize(hull, irm, true, ChVector<>(-2.500, 4.330, 8.660 - 5.7), ChVector<>(-1.250, 2.165, 4.330));
        spring_5->SetRestLength(pto_length);
        spring_5->SetSpringCoefficient(pto_stiffness);
        spring_5->SetDampingCoefficient(pto_damping);
        spring_5->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_5);

        // spring 6 between hull and irm
        auto spring_6 = chrono_types::make_shared<ChLinkTSDA>();
        spring_6->Initialize(hull, irm, true, ChVector<>(-2.500, -4.330, 8.660 - 5.7),
                             ChVector<>(-1.250, -2.165, 4.330));
        spring_6->SetRestLength(pto_length);
        spring_6->SetSpringCoefficient(pto_stiffness);
        spring_6->SetDampingCoefficient(pto_damping);
        spring_6->AddVisualShape(
            chrono_types::make_shared<ChSpringShape>(pto_radius, pto_resolution, pto_spring_turns));
        system.AddLink(spring_6);

        std::array<std::shared_ptr<ChLinkTSDA>, 6> springs = {spring_1, spring_2, spring_3,
                                                              spring_4, spring_5, spring_6};

        // auto my_hydro_inputs = std::make_shared<IrregularWave>();
        // my_hydro_inputs->wave_height         = 2.0;
        // my_hydro_inputs->wave_period         = 6.0;
        // my_hydro_inputs->simulation_duration = simulation_duration;
        // my_hydro_inputs->simulation_dt       = timestep;
        // my_hydro_inputs->ramp_duration       = simulation_duration*0.1;

        auto my_hydro_inputs                    = std::make_shared<RegularWave>(1);
        my_hydro_inputs->regular_wave_amplitude = 1.0;          // 0.095;
        my_hydro_inputs->regular_wave_omega     = 1.046778671;  // 1.427996661;

        // auto default_dont_add_waves = std::make_shared<NoWave>(1);

        // set up hydro forces
        std::vector<std::shared_ptr<ChBody>> bodies;
        bodies.push_back(hull);
        TestHydro hydro_forces(bodies, h5_fname);
        hydro_forces.AddWaves(my_hydro_inputs);

        //// set up free surface from a mesh
        // auto fse_plane = chrono_types::make_shared<ChBody>();
        // fse_plane->SetPos(ChVector<>(0, 0, 0));
        // fse_plane->SetBodyFixed(true);
        // fse_plane->SetCollide(false);
        // system.AddBody(fse_plane);

        // my_hydro_inputs->SetUpWaveMesh();
        // std::shared_ptr<ChBody> fse_mesh = chrono_types::make_shared<ChBodyEasyMesh>(  //
        //    my_hydro_inputs->GetMeshFile(),                                            // file name
        //    1000,                                                                      // density
        //    false,                                                                     // do not evaluate mass
        //    automatically true,                                                                      // create
        //    visualization asset false                                                                      // do not
        //    collide
        //);
        // fse_mesh->SetMass(1.0);
        // fse_mesh->SetPos_dt(my_hydro_inputs->GetWaveMeshVelocity());
        // system.Add(fse_mesh);
        // auto fse_prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        // fse_prismatic->Initialize(fse_plane, fse_mesh, ChCoordsys<>(ChVector<>(1.0, 0.0, 0.0),
        // Q_from_AngY(CH_C_PI_2))); system.AddLink(fse_prismatic);

        //// Create a visualization material
        // auto fse_texture = chrono_types::make_shared<ChVisualMaterial>();
        // fse_texture->SetDiffuseColor(ChColor(0.026f, 0.084f, 0.168f));
        // fse_texture->SetOpacity(0.1);
        // fse_mesh->GetVisualShape(0)->SetMaterial(0, fse_texture);

        // for profiling
        auto start = std::chrono::high_resolution_clock::now();

#ifdef HYDROCHRONO_HAVE_IRRLICHT
        if (visualization_on) {
            // create the irrlicht application for visualizing
            auto irrlicht_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            irrlicht_vis->AttachSystem(&system);
            irrlicht_vis->SetWindowSize(1280, 720);
            irrlicht_vis->SetWindowTitle("TALOS PTO");
            irrlicht_vis->SetCameraVertical(CameraVerticalDir::Z);
            irrlicht_vis->Initialize();
            irrlicht_vis->AddLogo();
            irrlicht_vis->AddSkyBox();
            irrlicht_vis->AddCamera(ChVector<>(0, -70, -10), ChVector<>(0, 0, -10));
            irrlicht_vis->AddTypicalLights();
            // irrlichtVis->EnableBodyFrameDrawing(true);
            // irrlichtVis->EnableLinkFrameDrawing(true);

            // add play/pause button
            bool buttonPressed = false;
            MyActionReceiver receiver(irrlicht_vis.get(), buttonPressed);
            irrlicht_vis->AddUserEventReceiver(&receiver);
            // ChSparseMatrix M;
            // main simulation loop
            while (irrlicht_vis->Run() && system.GetChTime() <= simulation_duration) {
                irrlicht_vis->BeginScene();
                irrlicht_vis->Render();
                irrlicht_vis->EndScene();
                if (buttonPressed) {
                    // system.GetMassMatrix(&M);
                    // std::cout << M << std::endl;
                    // step the simulation forwards
                    system.DoStepDynamics(timestep);
                    // append data to std vector
                    time_vector.push_back(system.GetChTime());
                    hull_surge.push_back(hull->GetPos().x());
                    hull_heave.push_back(hull->GetPos().z());
                    hull_pitch.push_back(hull->GetRot().Q_to_Euler123().y());
                    // force playback to be real-time
                    realtime_timer.Spin(timestep);

                    // Compute and store power absorbed by each damper and the total power absorbed
                    double total_power = 0.0;
                    for (int i = 0; i < springs.size(); ++i) {
                        double spring_velocity = springs[i]->GetVelocity();
                        double power_absorbed  = spring_velocity * spring_velocity * pto_damping;
                        power_absorbed_by_spring[i].push_back(power_absorbed);
                        total_power += power_absorbed;
                    }
                    total_power_absorbed.push_back(total_power);
                }
            }
        } else {
#endif  // #ifdef HYDROCHRONO_HAVE_IRRLICHT
            while (system.GetChTime() <= simulation_duration) {
                // append data to std vector
                time_vector.push_back(system.GetChTime());
                hull_surge.push_back(hull->GetPos().x());
                hull_heave.push_back(hull->GetPos().z());
                hull_pitch.push_back(hull->GetRot().Q_to_Euler123().y());
                // step the simulation forwards
                system.DoStepDynamics(timestep);

                // Compute and store power absorbed by each damper and the total power absorbed
                double total_power = 0.0;
                for (int i = 0; i < springs.size(); ++i) {
                    double spring_velocity = springs[i]->GetVelocity();
                    double power_absorbed  = spring_velocity * spring_velocity * pto_damping;
                    power_absorbed_by_spring[i].push_back(power_absorbed);
                    total_power += power_absorbed;
                }
                total_power_absorbed.push_back(total_power);
            }
#ifdef HYDROCHRONO_HAVE_IRRLICHT
        }
#endif
        if (save_data_on) {
            // Modify the directory where results are saved to include the stiffness value
            std::string stiffness_str = std::to_string(pto_stiffness);
            std::replace(stiffness_str.begin(), stiffness_str.end(), '.',
                         '_');  // Replace '.' with '_' because '.' is not allowed in folder names

            std::string base_path = "./results/talos/pto_k/" + stiffness_str + "/";

            if (!std::filesystem::exists(base_path)) {
                std::filesystem::create_directories(base_path);
            }

            std::ofstream output_file;
            output_file.open(base_path + "talos_reg.txt");

            std::ofstream power_output_file;
            power_output_file.open(base_path + "talos_reg_power.txt");

            // Write header for hull data
            output_file << "Time (s),"
                        << "Hull Surge (m),"
                        << "Hull Heave (m),"
                        << "Hull Pitch (radians)" << std::endl;

            // Write header for power data
            power_output_file << "Time (s)";
            for (int i = 1; i <= 6; ++i) {
                power_output_file << ","
                                  << "Power " + std::to_string(i) + " (W)";
            }
            power_output_file << ","
                              << "Total Power (W)" << std::endl;

            for (int i = 0; i < time_vector.size(); ++i) {
                // Write hull data
                output_file << std::fixed << std::setprecision(2) << time_vector[i] << "," << std::fixed
                            << std::setprecision(8) << hull_surge[i] << "," << std::fixed << std::setprecision(8)
                            << hull_heave[i] << "," << std::fixed << std::setprecision(8) << hull_pitch[i] << std::endl;

                // Write power absorbed by each damper and the total power absorbed
                power_output_file << std::fixed << std::setprecision(2) << time_vector[i];
                for (int j = 0; j < 6; ++j) {
                    power_output_file << "," << std::fixed << std::setprecision(8) << power_absorbed_by_spring[j][i];
                }
                power_output_file << "," << std::fixed << std::setprecision(8) << total_power_absorbed[i] << std::endl;
            }
            output_file.close();
            power_output_file.close();
        }
    }
    return 0;
}
