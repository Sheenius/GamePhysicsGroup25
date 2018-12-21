#include "SphereSystemSimulator.h"

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

SphereSystemSimulator::SphereSystemSimulator()
{
	m_iKernel = 1;
	m_fMass = 10.0f;
	m_fRadius = 0.05f;
	m_fDrawnRadius = 0.2f;
	m_fForceScaling = 10.0f;
	m_fDamping = 10.0f;
	m_iNumSpheres = 100;
	m_fGravity = 0;
	m_iAccelerator = NAIVEACC;
	position = Vec3(0, 0, 0);
	initialize(m_iNumSpheres);
	isRunning = true;
}

SphereSystemSimulator::~SphereSystemSimulator() {
	delete[] grid;
	delete m_pSphereSystemNaive;
	delete m_pSphereSystemGrid;
}

void SphereSystemSimulator::initialize(int n) {
	isRunning = true;
	spheres.clear();
	float x = -0.45f;
	float y = -0.45f;
	float z = -0.45f;
	float step = 0.9f / n / 3;
	/*m_iAccelerator = GRIDACC;
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			for (int k = 0; k < 1; k++) {
				Sphere sphere;
				sphere.position = Vec3(x + i * 0.08, y + j * 0.08, z + k * 0.008);
				sphere.velocity = Vec3(0, 0, 0);
				sphere.force = Vec3(0, 0, 0);
				spheres.push_back(sphere);
			}
		}
	}

	MuTime time;
	time.get();
	for (int i = 0; i < 100; i++) {
		this->simulateTimestep(0.001);
	}
	std::cout << "Time passed " << time.update().time << " milliseconds\n";*/

	for (int i = 0; i < n; i++) {
		Sphere sphere;
		sphere.position = Vec3(x, y, z);
		sphere.velocity = Vec3(0, 0, 0);
		sphere.force = Vec3(0, 0, 0);
		spheres.push_back(sphere);
		x += step;
		y += 2 * step;
		z += 3 * step;
	}
}

const char * SphereSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	reset();
	if (dontAddUIVariables) {
		return;
	}
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min = 0 max = 100 step = 0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "min = 0.005 max = 1 step = 0.005");
	TwAddVarRW(DUC->g_pTweakBar, "Drawn Radius", TW_TYPE_FLOAT, &m_fDrawnRadius, "min = 0.1 max = 1 step = 0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min = 0 max = 100 step = 0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Force Scaling", TW_TYPE_FLOAT, &m_fForceScaling, "min = 0 max = 1000 step = 0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min = 0 max = 10 step = 0.1");
	TwType TW_TYPE_ACCELERATOR = TwDefineEnumFromString("Collision method", "Naive, Grid");
	TwAddVarRW(DUC->g_pTweakBar, "CollisionMethod", TW_TYPE_ACCELERATOR, &m_iAccelerator, "");
}

void SphereSystemSimulator::reset()
{
	initialize(m_iNumSpheres);
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	if (m_iTestCase == 2) {
		
		m_pSphereSystemGrid->drawFrame(pd3dImmediateContext);
		m_pSphereSystemNaive->drawFrame(pd3dImmediateContext);
		return;
	}
	DUC->setUpLighting(Vec3(0, 0, 0), color, 1, color);
	for (vector<Sphere>::iterator it = spheres.begin(); it != spheres.end(); ++it) {
		DUC->drawSphere(position + it->position, Vec3(m_fRadius * m_fDrawnRadius));
	}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	delete m_pSphereSystemNaive;
	m_pSphereSystemNaive = nullptr;
	
	delete m_pSphereSystemGrid;
	m_pSphereSystemGrid = nullptr;
	m_iTestCase = testCase;
	reset();
	if (testCase == 0) {
		m_iAccelerator = NAIVEACC;
		color = Vec3(1, 0, 0);
	}
	else if (testCase == 1) {
		m_iAccelerator = GRIDACC;
		color = Vec3(0, 0, 1);
	}
	else if (testCase == 2) {
		m_pSphereSystemNaive = new SphereSystemSimulator;
		m_pSphereSystemNaive->notifyCaseChanged(0);
		m_pSphereSystemNaive->dontAddUIVariables = true;
		m_pSphereSystemNaive->initUI(DUC);

		m_pSphereSystemGrid = new SphereSystemSimulator;
		m_pSphereSystemGrid->dontAddUIVariables = true;
		m_pSphereSystemGrid->position = Vec3(0.0001, 0.0001, 0.0001);
		m_pSphereSystemGrid->notifyCaseChanged(1);
		m_pSphereSystemGrid->initUI(DUC);
		/////////////////////////////////
		//To Do                        //
		//CYNTHIA                      //
		/////////////////////////////////
	}
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

int SphereSystemSimulator::index(int x, int y, int z, int m) {
	return m + 100 * z + 100 * 100 * y + 100 * 100 * 100 * x;
}

void SphereSystemSimulator::naiveCollision() {
	for (int i = 0; i < spheres.size(); i++) {
		for (int j = i; j < spheres.size(); j++) {
			float d = norm(spheres[i].position - spheres[j].position);
			if (d < 2 * m_fRadius) {
				float force = m_Kernels[m_iKernel](d / (2 * m_fRadius));
				Vec3 direction = getNormalized(spheres[j].position - spheres[i].position);
				spheres[i].force += -force * direction * m_fForceScaling;
				spheres[j].force += force * direction * m_fForceScaling;
			}
		}
	}
}

void SphereSystemSimulator::gridCollision() {
	int n = (int) (1.0f / (2.0f * m_fRadius));
	if (n > 100) {
		naiveCollision();
		return;
	}

	memset(numberOfSpheresInCell, 0, sizeof(numberOfSpheresInCell));
	for (int i = 0; i < spheres.size(); i++) {
		spheres[i].positionInGrid.x = (int)((spheres[i].position.x + 0.5f) * n);
		spheres[i].positionInGrid.y = (int)((spheres[i].position.y + 0.5f) * n);
		spheres[i].positionInGrid.z = (int)((spheres[i].position.z + 0.5f) * n);
		int number = numberOfSpheresInCell[spheres[i].positionInGrid.x][spheres[i].positionInGrid.y][spheres[i].positionInGrid.z]++;
		grid[index(spheres[i].positionInGrid.x, spheres[i].positionInGrid.y, spheres[i].positionInGrid.z, number)] = i;
	}

	for (int i = 0; i < spheres.size(); i++) {
		Sphere toTest[100];
		int numToTest = 0;
		for (int j = -1; j <= 1; j++) {
			for (int k = -1; k <= 1; k++) {
				for (int l = -1; l <= 1; l++) {
					int x = spheres[i].positionInGrid.x + j;
					int y = spheres[i].positionInGrid.y + k;
					int z = spheres[i].positionInGrid.z + l;
					if (x < 0 || x >= 100 || y < 0 || y >= 100 || z < 0 || z >= 100) {
						continue;
					}
					for (int m = 0; m < numberOfSpheresInCell[x][y][z]; m++) {
						toTest[numToTest++] = spheres[grid[index(x, y, z, m)]];
					}
				}
			}
		}

		for (int j = 0; j < numToTest; ++j) {
			if (spheres[i].position.x == toTest[j].position.x && spheres[i].position.y == toTest[j].position.y && spheres[i].position.z == toTest[j].position.z) {
				continue;
			}
			float d = norm(spheres[i].position - toTest[j].position);
			if (d < 2 * m_fRadius) {
				float force = m_Kernels[m_iKernel](d / (2 * m_fRadius));
				Vec3 direction = getNormalized(toTest[j].position - spheres[i].position);
				spheres[i].force += -force * direction * m_fForceScaling;
			}
		}
	}
}

void SphereSystemSimulator::calculateForces() {
	for (vector<Sphere>::iterator it = spheres.begin(); it != spheres.end(); ++it) {
		it->force = it->force = Vec3(0, -m_fGravity, 0) * m_fMass - m_fDamping * it->velocity;
		float d;
		d = abs(it->position.x - (-0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(1, 0, 0) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
		d = abs(it->position.x - (0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(-1, 0, 0) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
		d = abs(it->position.y - (-0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(0, 1, 0) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
		d = abs(it->position.y - (0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(0, -1, 0) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
		d = abs(it->position.z- (-0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(0, 0, 1) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
		d = abs(it->position.z - (0.5));
		if (d < 2 * m_fRadius) {
			it->force += m_fForceScaling * Vec3(0, 0, -1) * m_Kernels[m_iKernel](d / (2 * m_fRadius));
		}
	}

	if (m_iAccelerator == NAIVEACC) {
		naiveCollision();
	}
	else if (m_iAccelerator == GRIDACC) {
		gridCollision();
	}
	else {
		naiveCollision();
	}
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 2) {
		m_pSphereSystemNaive->simulateTimestep(timeStep);
		m_pSphereSystemGrid->simulateTimestep(timeStep);
		return;
	}
	calculateForces();
	for (vector<Sphere>::iterator it = spheres.begin(); it != spheres.end(); ++it) {
		it->velocity += it->force / m_fMass * timeStep;
		it->position += it->velocity * timeStep;
		if (it->position.x < -0.5) {
			it->position.x = -0.5;
		}
		if (it->position.x > 0.5) {
			it->position.x = 0.5;
		}
		if (it->position.y < -0.5) {
			it->position.y = -0.5;
		}
		if (it->position.y > 0.5) {
			it->position.y = 0.5;
		}
		if (it->position.z < -0.5) {
			it->position.z = -0.5;
		}
		if (it->position.z > 0.5) {
			it->position.z = 0.5;
		}
	}
}

void SphereSystemSimulator::onClick(int x, int y)
{
}

void SphereSystemSimulator::onMouse(int x, int y)
{
}
