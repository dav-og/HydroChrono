#include <Eigen>

using Eigen::VectorXd;

// class that does the math
class GenHydroLib {
public:
	VectorXd ComputeForceHyrostatics() {
		VectorXd displacement = KinematicsRef.GetPos() - FileInfo.GetEquilibrium();
		// do compute force math
		VectorXd result;
		return result;
	}
	// other computation functions for other forces, multibody, same as hydrochrono

	GenericKinematics KinematicsRef;
	GenericHydroInfo Info; 
private:
};

// todo pick different name
class GenericKinematics {
	virtual VectorXd GetPos() = 0;
};

class GenericHydroForce {
	virtual void SetForce(VectorXd force) = 0;
};

class GenericHydroInfo {
public:
	virtual void SetEquilibrium(VectorXd eq) = 0;
	virtual VectorXd GetEquilibrium() = 0;
private:
	VectorXd equilibrium;
	MatrixXd stiffness;
	// all other info from h5 used to calculate forces
};

// todo move these classes out of this file
#include "chrono/physics/ChBody.h"
class ChronoKinematics : public GenericKinematics {
public:
	std::shared_ptr<chrono::ChBody> singlebody; // todo generalize to multibody later

	GenericKinematics(std::shared_ptr<chrono::ChBody> singlebody) {
		this->singlebody = singlebody;
	}

	VectorXd GetPos() {
		VectorXd pos = singlebody->GetPos();
		return pos; // todo I don't think this will work here since pos is a local variable
	}
	// also have get rotation, velocity, angular velocity, etc
};

#include "chrono/physics/ChForce.h"
class ChronoHydroForce : public GenericHydroForce {
	chrono::ChForce force;
	void SetForce(VectorXd force) {
		// set chrono force on body = to argument force here
		this->force.SetForce(force); // placeholder, this step is actually more complicated
	}
};

class H5FileInfo : public GenericHydroInfo {
	void SetEquilibrium(VectorXd eq) {
		// read the info from h5 file and store it as a member variable
		// in this case the variable eq is never used
	}
	VectorXd GetEquilibrium() {
		return equilibrium;
	}
};

class manualInfoEntry : public GenericHydroInfo {
	void SetEquilibrium(VectorXd eq) {
		// an example of a time the variable eq is used
		equilibrium = eq;
	}
	VectorXd GetEquilibrium() {
		return equilibrium; // todo will this function always be the same?
	}
};
