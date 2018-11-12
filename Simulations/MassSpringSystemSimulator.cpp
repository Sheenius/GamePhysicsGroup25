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
	TwAddVarRW(DUC->g_pTweakBar, "SimulationMethod", TW_TYPE_TESTCASE, &m_iIntegrator, "");
	if (simulationMethodReadOnly) {
		TwDefine("TweakBar/SimulationMethod readonly = true");
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(0,0,0), MASSPOINT_COLOR, 1, MASSPOINT_COLOR);
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		DUC->drawSphere(it->position, Vec3(MASSPOINT_SIZE));
	}
	for (vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
		DUC->beginLine();
		Vec3 pos1 = masspoints[it->masspoint1].position;
		Vec3 pos2 = masspoints[it->masspoint2].position;
		DUC->drawLine(pos1, SPRING_COLOR, pos2, SPRING_COLOR);
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	simulationMethodReadOnly = false;


	if (testCase < 3) {
		setUpBasicTestscene();

		TwDefine("TweakBar/Timestep visible = false");
	}
	else {
		setUpComplexTestScene();
	}

	switch (testCase) {
	case 0:
		demo1(0);
		break;
	case 1:
		timestep = 0.005;
		m_iIntegrator = EULER;
		simulationMethodReadOnly = true;
		break;
	case 2:
		timestep = 0.005;
		m_iIntegrator = MIDPOINT;
		simulationMethodReadOnly = true;
		break;
	case 3:
		break;
	}
}

void MassSpringSystemSimulator::setUpBasicTestscene() {
	masspoints.clear();
	springs.clear();

	int masspoint1, masspoint2;
	masspoint1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	masspoint2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(masspoint1, masspoint2, 1);


	setMass(10);
	setStiffness(40);
	setDampingFactor(0);
}

void MassSpringSystemSimulator::setUpComplexTestScene() {
	masspoints.clear();
	springs.clear();

	int mp1, mp2, mp3, mp4, mp5, mp6, mp7, mp8, mp9, mp10;
	mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	mp3 = addMassPoint(Vec3(0, 4, 0), Vec3(-1, 0, 0), false);
	mp4 = addMassPoint(Vec3(0, 6, 0), Vec3(1, 0, 0), false);
	mp5 = addMassPoint(Vec3(0, 8, 0), Vec3(-1, 0, 0), false);
	mp6 = addMassPoint(Vec3(0, 10, 0), Vec3(1, 0, 0), false);
	mp7 = addMassPoint(Vec3(0, 12, 0), Vec3(-1, 0, 0), false);
	mp8 = addMassPoint(Vec3(0, 14, 0), Vec3(1, 0, 0), false);
	mp9 = addMassPoint(Vec3(0, 16, 0), Vec3(-1, 0, 0), false);
	mp10 = addMassPoint(Vec3(0, 18, 0), Vec3(1, 0, 0), true);

	addSpring(mp1, mp2, 1);
	addSpring(mp2, mp3, 1);
	addSpring(mp3, mp4, 1);
	addSpring(mp4, mp5, 1);
	addSpring(mp5, mp6, 1);
	addSpring(mp6, mp7, 1);
	addSpring(mp7, mp8, 1);
	addSpring(mp8, mp9, 1);
	addSpring(mp9, mp10, 1);
	//addMassPoint(Vec3(), Vec3(), false);

	gravity = Vec3(0, -10, 0);
	setMass(10);
	setStiffness(200);
	setDampingFactor(0.01);
}

void MassSpringSystemSimulator::demo1(int i) {
	m_iIntegrator = EULER;
	simulateTimestep(0.1);
	cout << "EULER:\n";
	printSolution();

	setUpBasicTestscene();
	m_iIntegrator = MIDPOINT;
	simulateTimestep(0.1);
	cout << "\nMIDPOINT:\n";
	printSolution();
}

void MassSpringSystemSimulator::printSolution() {
	for (int i = 0; i < masspoints.size(); i++) {
		cout << "Masspoint " << i << ":\n";
		cout << "  x = " << masspoints[i].position.x << "\n";
		cout << "  y = " << masspoints[i].position.y << "\n";
		cout << "  z = " << masspoints[i].position.z << "\n";
	}	
}



void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	
}

void MassSpringSystemSimulator::calculateForces() {
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		it->force = gravity * m_fMass - m_fDamping * it->velocity;
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
	if (timestep != 0 && m_iTestCase < 3) {
		timeStep = timestep;
	}
	//MassSpringSystemSimulator::externalForcesCalculations(timeStep)
	calculateForces();
	if (m_iIntegrator == EULER) {
		for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
			if (!it->isFixed) {
				it->position += it->velocity * timeStep;
			}
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
			if (!masspoints[i].isFixed) {
				masspoints[i].position = oldPositions[i] + masspoints[i].velocity * timeStep;
			}
			else {
				masspoints[i].position = oldPositions[i];
			}
			masspoints[i].velocity = oldVelocities[i] + masspoints[i].force / m_fMass * timeStep;
		}
	}
	else if (m_iIntegrator == LEAPFROG) {
		for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
			it->velocity += it->force / m_fMass * timeStep;
			if (!it->isFixed){
				it->position += it->velocity * timeStep;
			}
		}
	}

	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); it++) {
		if (it->position.y - MASSPOINT_SIZE < -1.0) {
			it->position.y = -1.0 + MASSPOINT_SIZE;
			it->velocity = reflectVector(it->velocity, Vec3(0, 1, 0));
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

