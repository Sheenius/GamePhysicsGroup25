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
	bool isFixed;
};

struct Spring {
	int masspoint1;
	int masspoint2;
	float initialLength;
};

std::vector<Masspoint> masspoints;
std::vector<Spring> springs;


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
	Simulator::DUC = DUC;
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(0,0,0), Vec3(), 0, Vec3(1, 0, 0));
	for (std::vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		DUC->drawSphere(it->position, Vec3(1, 1, 1));
	}
	for (std::vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
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
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {

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

