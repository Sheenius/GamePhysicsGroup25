#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = 0;
}


void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

void MassSpringSystemSimulator::setIntegrator(int integrator) {
	m_iIntegrator = integrator;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {

}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {

}

int MassSpringSystemSimulator::getNumberOfMassPoints() {

}

int MassSpringSystemSimulator::getNumberOfSprings() {

}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {

}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {

}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}




//UI Functions


//Print solution in Commandline
const char* MassSpringSystemSimulator::getTestCasesStr() {

}


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {

}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {

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

