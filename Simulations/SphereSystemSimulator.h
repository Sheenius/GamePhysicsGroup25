#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
//#include "spheresystem.h", add your sphere system header file

#define NAIVEACC 0
#define GRIDACC 1

class SphereSystemSimulator:public Simulator{
public:
	// Construtors
	SphereSystemSimulator();
	~SphereSystemSimulator();
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
	
protected:
	// Attributes
	boolean dontAddUIVariables = false;
	Vec3 color;
	boolean isRunning;
	Vec3 externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fMass;
	float m_fRadius;
	float m_fDrawnRadius;
	float m_fForceScaling;
	float m_fDamping;
	int   m_iNumSpheres;
	float m_fGravity;
	Vec3 position;
	struct Position {
		int x;
		int y;
		int z;
	};
	struct Sphere {
		Vec3 position;
		Vec3 velocity;
		Vec3 force;
		Position positionInGrid;
	};
	vector<Sphere> spheres;

	int * grid = new int[100*100*100*10];
	int numberOfSpheresInCell[100][100][100];
	
	int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static std::function<float(float)> m_Kernels[5];
	
	int   m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	
	//SphereSystem * m_pSphereSystem; // add your own sphere system member!
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	SphereSystemSimulator * m_pSphereSystemNaive = nullptr;
	SphereSystemSimulator * m_pSphereSystemGrid = nullptr;

	void initialize(int n);
	void calculateForces();
	void naiveCollision();
	void gridCollision();
	int index(int x, int y, int z, int m);

};

#endif