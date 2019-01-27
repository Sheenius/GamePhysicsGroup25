#pragma once
#include "Simulator.h"
#include "collisionDetect.h"
class RigidBodySpringSystemSimulator : public Simulator {
public:
	//Constructors
	RigidBodySpringSystemSimulator();
	~RigidBodySpringSystemSimulator();

	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
private:
	void addRigidBody(Vec3 position, Vec3 size, double mass);
	int addMassPoint(Vec3 position, Vec3 velocity, double massInverse, int rigidBody);
	void addSpring(int masspoint1, int masspoint2, double initialLength, double stiffness);



	const Vec3 BOX_COLOR = Vec3(18.0 / 256, 10.0 / 256, 143.0 / 256);
	const double MASSPOINT_SIZE = 0.06;
	const Vec3 MASSPOINT_COLOR = Vec3(1, 0, 0);
	const Vec3 SPRING_COLOR = Vec3(0, 1, 0);

	Vec3 gravity = Vec3(0, -1, 0);
	double damping = 0.1;
	const double BOUNCINESS = 1;

	struct Rigidbody {
		Vec3 position;
		Vec3 size;
		Quaternion<double> orientation;
		double massInverse;
		Vec3 linearVelocity;
		Vec3 angularMomentum;
		matrix4x4<double> inertiaTensor;

		Vec3 force;
		Vec3 torque;
	};
	vector<Rigidbody> rigidbodies;

	struct Masspoint {
		double massInverse;
		Vec3 position;
		Vec3 velocity;
		Vec3 force;
		int rigidBody;
	};
	vector<Masspoint> masspoints;

	struct Spring {
		int masspoint1;
		int masspoint2;
		double initialLength;
		double stiffness;
	};
	vector<Spring> springs;

	struct Force {
		int rigidbody;
		Vec3 position;
		Vec3 force;
	};
	vector<Force> forces;

	matrix4x4<double> calculateInertiaTensor(double massInverse, Vec3 size);
	void setOrientationOf(int i, Quat orientation);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void collisionDetection();

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

