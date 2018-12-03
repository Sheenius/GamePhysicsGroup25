#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void printSolution();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem;

	const Vec3 BOX_COLOR = Vec3(18.0/256, 10.0/256, 143.0/256);

	Vec3 gravity = Vec3(0, 0, 0);

	struct Rigidbody {
		Vec3 position;
		Vec3 size;
		Quaternion<double> orientation;
		double mass;
		Vec3 linearVelocity;
		Vec3 angularMomentum;
		matrix4x4<double> inertiaTensor;

		Vec3 force;
		Vec3 torque;
	};

	vector<Rigidbody> rigidbodies;

	struct Force {
		int rigidbody;
		Vec3 position;
		Vec3 force;
	};

	vector<Force> forces;

	const double BOUNCINESS = 0.7;

	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	//Methods
	void demo1();
	void demo2();
	void demo3();
	void demo4();
	matrix4x4<double> RigidBodySystemSimulator::calculateInertiaTensor(double mass, Vec3 size);
	void collisionDetection();
	};
#endif