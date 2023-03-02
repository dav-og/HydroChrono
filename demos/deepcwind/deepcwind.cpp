#include "../../src/hydro_forces.h"
//#include "./src/hydro_forces.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include "chrono/core/ChRealtimeStep.h"
#include <iomanip> // std::setprecision
#include <chrono> // std::chrono::high_resolution_clock::now
#include <vector> // std::vector<double>
#include "chrono/physics/ChLinkRSDA.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

class MyActionReceiver : public IEventReceiver {
public:
	MyActionReceiver(ChVisualSystemIrrlicht* vsys, bool& buttonPressed)
		: pressed(buttonPressed) {
		// store pointer application
		vis = vsys;

		// ..add a GUI button to control pause/play
		pauseButton = vis->GetGUIEnvironment()->addButton(rect<s32>(510, 20, 650, 35));
		buttonText = vis->GetGUIEnvironment()->addStaticText(L"Paused", rect<s32>(560, 20, 600, 35), false);
	}

	bool OnEvent(const SEvent& event) {
		// check if user clicked button
		if (event.EventType == EET_GUI_EVENT) {
			switch (event.GUIEvent.EventType) {
			case EGET_BUTTON_CLICKED:
				pressed = !pressed;
				if (pressed) {
					buttonText->setText(L"Playing");
				}
				else {
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

int main(int argc, char* argv[]) {
	//auto start = std::chrono::high_resolution_clock::now();
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	// system/solver settings
	ChSystemNSC system;
	system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
	double timestep = 0.08;
	system.SetTimestepperType(ChTimestepper::Type::HHT);
	system.SetSolverType(ChSolver::Type::GMRES);
	system.SetSolverMaxIterations(300);  // the higher, the easier to keep the constraints satisfied.
	system.SetStep(timestep);
	ChRealtimeStepTimer realtime_timer;
	double simulationDuration = 1000.0;

	// some io/viz options
	bool visualizationOn = true;
	bool profilingOn = true;
	bool saveDataOn = true;
	std::vector<double> time_vector;
	std::vector<double> body_pitch;
	std::vector<double> float_drift_position;
	std::vector<double> plate_heave_position;

	// set up body from a mesh
	if (!std::filesystem::exists("../../HydroChrono/demos/deepcwind/geometry/deepcwind.obj")) {
		std::cout << "File " << std::filesystem::absolute(GetChronoDataFile("../../HydroChrono/demos/deepcwind/geometry/deepcwind.obj").c_str()) << " does not exist" << std::endl;
		return 0;
	}
	//std::cout << "Attempting to open mesh file: " << std::filesystem::absolute(GetChronoDataFile("../../HydroChrono/meshFiles/float.obj").c_str()) << std::endl;
	std::shared_ptr<ChBody> body = chrono_types::make_shared<ChBodyEasyMesh>(                   //
		GetChronoDataFile("../../HydroChrono/demos/deepcwind/geometry/deepcwind.obj").c_str(),                 // file name
		0,                                                                                        // density
		false,                                                                                    // do not evaluate mass automatically
		true,                                                                                     // create visualization asset
		false                                                                                     // collisions
		);

	// define the body's initial conditions
	system.Add(body);
	body->SetNameString("body1");
	auto cg = ChVector<>(0.0, 0.0, -7.53);
	auto offset = ChVector<>(0.0, 0.0, 0.0);
	body->SetPos(cg + offset);
	double ang_rad = -3.95 * CH_C_PI / 180.0;
	body->SetRot(Q_from_AngAxis(ang_rad, VECT_Y));
	body->SetMass(1.419625e7);
	body->SetInertiaXX(ChVector<>(1.2898e10, 1.2851e10, 1.4189e10)); 

	auto ground = chrono_types::make_shared<ChBody>();
	system.AddBody(ground);
	ground->SetPos(cg);
	ground->SetRot(Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
	ground->SetIdentifier(-1);
	ground->SetBodyFixed(true);
	ground->SetCollide(false);

	// define damping in pitch
	auto rot_damp = chrono_types::make_shared<ChLinkRSDA>();
	// need to set damping to 31 MN-m/(rad/s)
	rot_damp->SetDampingCoefficient(31e6);
	// puts Z axis for link into screen, keeping x axis the same (to the right)
	ChQuaternion<> rev_rot = Q_from_AngAxis(CH_C_PI / 2.0, VECT_X); 
	rot_damp->Initialize(body, ground, false, ChCoordsys(cg, rev_rot), ChCoordsys(cg, rev_rot));
	system.AddLink(rot_damp);

	// define wave parameters 
	HydroInputs my_hydro_inputs;
	my_hydro_inputs.mode = noWaveCIC;
	my_hydro_inputs.regular_wave_amplitude = 1.0;
	my_hydro_inputs.regular_wave_omega = 2.10;

	// attach hydrodynamic forces to body
	std::vector<std::shared_ptr<ChBody>> bodies;
	bodies.push_back(body);
	TestHydro blah(bodies, "../../HydroChrono/demos/deepcwind/hydroData/deepcwind.h5", my_hydro_inputs);

	//// Debug printing added mass matrix and system mass matrix
	//ChSparseMatrix M;
	//system.GetMassMatrix(&M);
	//std::cout << M << std::endl;

	// for profiling
	auto start = std::chrono::high_resolution_clock::now();

	if (visualizationOn) {
		// create the irrlicht application for visualizing
		auto irrlichtVis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
		irrlichtVis->AttachSystem(&system);
		irrlichtVis->SetWindowSize(1280, 720);
		irrlichtVis->SetWindowTitle("DeepCwind Verification");
		irrlichtVis->SetCameraVertical(CameraVerticalDir::Z);
		irrlichtVis->Initialize();
		irrlichtVis->AddLogo();
		irrlichtVis->AddSkyBox();
		// camera position and where it points
		irrlichtVis->AddCamera(ChVector<>(0, -70, -10), ChVector<>(0, 0, -10)); 
		irrlichtVis->AddTypicalLights();

		// add play/pause button
		bool buttonPressed = false;
		MyActionReceiver receiver(irrlichtVis.get(), buttonPressed);
		irrlichtVis->AddUserEventReceiver(&receiver);

		// main simulation loop
		while (irrlichtVis->Run() && system.GetChTime() <= simulationDuration) {
			irrlichtVis->BeginScene();
			irrlichtVis->Render();
			irrlichtVis->EndScene();
			irrlichtVis->EnableBodyFrameDrawing(50);
			irrlichtVis->EnableLinkFrameDrawing(50);
			if (buttonPressed) {
				// step the simulation forwards
				system.DoStepDynamics(timestep);
				// append data to std vector
				time_vector.push_back(system.GetChTime());
				body_pitch.push_back(body->GetRot().Q_to_Euler123().y());
				//float_drift_position.push_back(body->GetPos().x());
				//plate_heave_position.push_back(plate_body2->GetPos().z());
				// force playback to be real-time
				realtime_timer.Spin(timestep);
			}
		}
	}
	else {
		int frame = 0;
		while (system.GetChTime() <= simulationDuration) {
			// append data to std vector
			time_vector.push_back(system.GetChTime());
			body_pitch.push_back(body->GetPos().z());
			//float_drift_position.push_back(body->GetPos().x());
			//plate_heave_position.push_back(plate_body2->GetPos().z());

			// step the simulation forwards
			system.DoStepDynamics(timestep);

			frame++;
		}
	}

	// for profiling
	auto end = std::chrono::high_resolution_clock::now();
	unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	if (profilingOn) {
		std::ofstream profilingFile;
		profilingFile.open("./results/deepcwind/decay/duration_ms.txt");
		if (!profilingFile.is_open()) {
			if (!std::filesystem::exists("./results/deepcwind/decay")) {
				std::cout << "Path " << std::filesystem::absolute("./results/deepcwind/decay") 
					<< " does not exist, creating it now..." << std::endl;
				std::filesystem::create_directory("./results");
				std::filesystem::create_directory("./results/deepcwind");
				std::filesystem::create_directory("./results/deepcwind/decay");
				profilingFile.open("./results/deepcwind/decay/duration_ms.txt");
				if (!profilingFile.is_open()) {
					std::cout << "Still cannot open file, ending program" << std::endl;
					return 0;
				}
			}
		}
		profilingFile << duration << "\n";
		profilingFile.close();
	}

	if (saveDataOn) {
		std::ofstream outputFile;
		outputFile.open("./results/deepcwind/decay/deepcwind_decay.txt");
		if (!outputFile.is_open()) {
			if (!std::filesystem::exists("./results/deepcwind/decay")) {
				std::cout << "Path " << std::filesystem::absolute("./results/deepcwind/decay") 
					<< " does not exist, creating it now..." << std::endl;
				std::filesystem::create_directory("./results");
				std::filesystem::create_directory("./results/deepcwind");
				std::filesystem::create_directory("./results/deepcwind/decay");
				outputFile.open("./results/deepcwind/decay/deepcwind_decay.txt");
				if (!outputFile.is_open()) {
					std::cout << "Still cannot open file, ending program" << std::endl;
					return 0;
				}
			}
		}
		outputFile << std::left << std::setw(10) << "Time (s)"
			<< std::right << std::setw(16) << "Float Heave (m)"
			//<< std::right << std::setw(16) << "Plate Heave (m)"
			//<< std::right << std::setw(16) << "Float Drift (x) (m)"
			<< std::endl;
		for (int i = 0; i < time_vector.size(); ++i)
			outputFile << std::left << std::setw(10) << std::setprecision(2) << std::fixed 
			<< time_vector[i] << std::right << std::setw(16) << std::setprecision(4) 
			<< std::fixed << body_pitch[i]
			//<< std::right << std::setw(16) << std::setprecision(4) << std::fixed << plate_heave_position[i]
			//<< std::right << std::setw(16) << std::setprecision(4) << std::fixed << float_drift_position[i]
			<< std::endl;
		outputFile.close();
	}
	return 0;
}