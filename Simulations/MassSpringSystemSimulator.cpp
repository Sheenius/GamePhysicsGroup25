#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = 0;
}

struct Masspoint {
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;
};

struct Spring {
	int masspoint1;
	int masspoint2;
	double initialLength;
};

vector<Masspoint> masspoints;
vector<Spring> springs;


void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	Masspoint masspoint;
	masspoint.position = position;
	masspoint.velocity = velocity;
	masspoint.force = Vec3();
	masspoint.isFixed = isFixed;
	masspoints.push_back(masspoint);
	return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring spring;
	spring.masspoint1 = masspoint1;
	spring.masspoint2 = masspoint2;
	spring.initialLength = initialLength;
	springs.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return masspoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return masspoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}




//UI Functions


//Print solution in Commandline
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
	
	TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Simulation Method", "Euler, Leapfrog, Midstep");
	TwAddVarRW(DUC->g_pTweakBar, "Simulation Method", TW_TYPE_TESTCASE, &m_iIntegrator, "");
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(0,0,0), Vec3(1, 0, 0), 1, Vec3(1, 0, 0));
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		DUC->drawSphere(it->position, Vec3(0.1, 0.1, 0.1));
	}
	for (vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
		DUC->beginLine();
		Vec3 pos1 = masspoints[it->masspoint1].position;
		Vec3 pos2 = masspoints[it->masspoint2].position;
		Vec3 color = Vec3(0, 1, 0);
		DUC->drawLine(pos1, color, pos2, color);
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	masspoints.clear();
	springs.clear();

	if (testCase < 3) {
		int masspoint1, masspoint2;
		masspoint1 = MassSpringSystemSimulator::addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		masspoint2 = MassSpringSystemSimulator::addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		MassSpringSystemSimulator::addSpring(masspoint1, masspoint2, 1);


		MassSpringSystemSimulator::setMass(10);
		MassSpringSystemSimulator::setStiffness(40);
		MassSpringSystemSimulator::setDampingFactor(0);
	}
}



void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	
}

void MassSpringSystemSimulator::calculateForces() {
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		it->force = 0;
	}
	for (vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
		Spring spring = *it;
		Masspoint mp1 = masspoints[spring.masspoint1];
		Masspoint mp2 = masspoints[spring.masspoint2];
		double currentLength = norm(mp1.position - mp2.position);
		double undirectedForce = -m_fStiffness * (currentLength - spring.initialLength);
		Vec3 force = undirectedForce * getNormalized(mp1.position - mp2.position);
		masspoints[spring.masspoint1].force += force;
		masspoints[spring.masspoint2].force -= force;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//MassSpringSystemSimulator::externalForcesCalculations(timeStep)
	calculateForces();
	if (m_iIntegrator == EULER) {
		for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
			it->position += it->velocity * timeStep;
			it->velocity += it->force / m_fMass * timeStep;
		}
	}
	else if (m_iIntegrator == MIDPOINT) {
		vector<Vec3> oldPositions;
		vector<Vec3> oldVelocities;
		for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
			Vec3 oldPos = it-> position;
			oldPositions.push_back(oldPos);
			Vec3 oldVelocity = it->velocity;
			oldVelocities.push_back(oldVelocity);
			it->position += it->velocity * timeStep / 2.0;
			it->velocity += it->force / m_fMass * timeStep / 2.0;
		}
		calculateForces();
		for (int i = 0; i < masspoints.size(); i++) {
			masspoints[i].position = oldPositions[i] + masspoints[i].velocity * timeStep;
			masspoints[i].velocity = oldVelocities[i] + masspoints[i].force / m_fMass * timeStep;
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

