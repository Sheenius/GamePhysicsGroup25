#include "RigidBodySpringSystemSimulator.h"



RigidBodySpringSystemSimulator::RigidBodySpringSystemSimulator()
{
}


RigidBodySpringSystemSimulator::~RigidBodySpringSystemSimulator()
{
}

const char * RigidBodySpringSystemSimulator::getTestCasesStr()
{
	return "Squidwart";
}

void RigidBodySpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void RigidBodySpringSystemSimulator::reset()
{
	rigidbodies.clear();
	masspoints.clear();
	springs.clear();
	forces.clear();
	

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	//Draw RigidBodies
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, BOX_COLOR);
	for (vector<Rigidbody>::iterator it = rigidbodies.begin(); it != rigidbodies.end(); ++it) {
		matrix4x4<double> scaleMatrix = matrix4x4<double>(it->size.x, 0, 0, 0,
			0, it->size.y, 0, 0,
			0, 0, it->size.z, 0,
			0, 0, 0, 1);
		scaleMatrix.initScaling(it->size.x, it->size.y, it->size.z);
		matrix4x4<double> rotationMatrix = it->orientation.getRotMat();
		matrix4x4<double> translationMatrix = matrix4x4<double>(1, 0, 0, it->position.x,
			0, 1, 0, it->position.y,
			0, 0, 1, it->position.z,
			0, 0, 0, 1);
		translationMatrix.initTranslation(it->position.x, it->position.y, it->position.z);
		matrix4x4<double> objToWorldMatrix = scaleMatrix * rotationMatrix * translationMatrix;
		DUC->drawRigidBody(objToWorldMatrix);
	}

	//Draw Masspoints
	DUC->setUpLighting(Vec3(0, 0, 0), MASSPOINT_COLOR, 1, MASSPOINT_COLOR);
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		Vec3 position = it->position;
		if (it->rigidBody >= 0) {
			position = rigidbodies[it->rigidBody].orientation.getRotMat().transformVector(position);
			position += rigidbodies[it->rigidBody].position;
		}
		DUC->drawSphere(position, Vec3(MASSPOINT_SIZE));
	}

	//Draw Springs
	for (vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
		DUC->beginLine();
		Vec3 position1 = masspoints[it->masspoint1].position;
		if (masspoints[it->masspoint1].rigidBody >= 0) {
			position1 = rigidbodies[masspoints[it->masspoint1].rigidBody].orientation.getRotMat().transformVector(position1);
			position1 += rigidbodies[masspoints[it->masspoint1].rigidBody].position;
		}
		Vec3 position2 = masspoints[it->masspoint2].position;
		if (masspoints[it->masspoint2].rigidBody >= 0) {
			position2 = rigidbodies[masspoints[it->masspoint2].rigidBody].orientation.getRotMat().transformVector(position2);
			position2 += rigidbodies[masspoints[it->masspoint2].rigidBody].position;
		}
		DUC->drawLine(position1, SPRING_COLOR, position2, SPRING_COLOR);
		DUC->endLine();
	}
}

void RigidBodySpringSystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	/*addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 0);
	addRigidBody(Vec3(-3, -0.5, 0), Vec3(0.25, 0.25, 0.25), 0.1);
	matrix4x4<double> rotationMatrix = matrix4x4<double>();
	//rotationMatrix.initRotationY(-45);
	rotationMatrix.initRotationXYZ(45, -45, 0);
	Quaternion<double> rotationQuaternion = Quaternion<double>(rotationMatrix);
	setOrientationOf(0, rotationQuaternion);

	addMassPoint(Vec3(-2, 0.5, 0), Vec3(0, 0, 0), 1, -1);
	addMassPoint(Vec3(-1, 1.5, 0), Vec3(0, 0, 0), 1, -1);
	addMassPoint(Vec3(0, 2.5, 0), Vec3(0, 0, 0), 0, -1);

	float sqrt2 = sqrt(2);
	addSpring(0, 1, sqrt2, 20);
	addSpring(1, 2, sqrt2, 20);
	addSpring(2, 3, sqrt2, 20);

	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 1);
	addRigidBody(Vec3(3, -0.5, 0), Vec3(0.25, 0.25, 0.25), 0.1);

	addMassPoint(Vec3(2, 0.5, 0), Vec3(0, 0, 0), 1, -1);
	addMassPoint(Vec3(1, 1.5, 0), Vec3(0, 0, 0), 1, -1);

	addSpring(4, 5, sqrt2, 20);
	addSpring(5, 6, sqrt2, 20);
	addSpring(6, 3, sqrt2, 20);

	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 2);
	addRigidBody(Vec3(0, -0.5, 3), Vec3(0.25, 0.25, 0.25), 0.1);

	addMassPoint(Vec3(0, 0.5, 2), Vec3(0, 0, 0), 1, -1);
	addMassPoint(Vec3(0, 1.5, 1), Vec3(0, 0, 0), 1, -1);

	addSpring(7, 8, sqrt2, 20);
	addSpring(8, 9, sqrt2, 20);
	addSpring(9, 3, sqrt2, 20);

	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 3);
	addRigidBody(Vec3(0, -0.5, -3), Vec3(0.25, 0.25, 0.25), 0.1);

	addMassPoint(Vec3(0, 0.5, -2), Vec3(0, 0, 0), 1, -1);
	addMassPoint(Vec3(0, 1.5, -1), Vec3(0, 0, 0), 1, -1);

	addSpring(10, 11, sqrt2, 20);
	addSpring(11, 12, sqrt2, 20);
	addSpring(12, 3, sqrt2, 20);*/

	float rigidBodyMass = 0.05;

	addMassPoint(Vec3(0, 7.5, 0), Vec3(0, 0, 0), 0, -1);

	addRigidBody(Vec3(-5, 7.5, 0), Vec3(0.25, 0.25, 0.25), rigidBodyMass);
	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 0);

	addRigidBody(Vec3(5, 7.5, 0), Vec3(0.25, 0.25, 0.25), rigidBodyMass);
	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 1);

	addRigidBody(Vec3(0, 10.5, -4), Vec3(0.25, 0.25, 0.25), rigidBodyMass);
	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 2);

	addRigidBody(Vec3(0, 10.5, 4), Vec3(0.25, 0.25, 0.25), rigidBodyMass);
	addMassPoint(Vec3(0.0001, 0.0001, 0.0001), Vec3(0, 0, 0), 1, 3);

	float stiffness = 200;

	for (int i = 0; i < 4; i++) {
		Vec3 increment = (masspoints[0].position - rigidbodies[i].position) / 12;
		Vec3 position = rigidbodies[i].position + increment;
		addSpring(i + 1, masspoints.size(), norm(increment), stiffness);
		for (int j = 0; j < 11; j++) {
			addMassPoint(position, Vec3(0, 0, 0), 1, -1);
			if (j < 10) {
				addSpring(masspoints.size() - 1, masspoints.size(), norm(increment), stiffness);
			}
			position += increment;
		}
		addSpring(masspoints.size() - 1, 0, norm(increment), stiffness);
	}

}

void RigidBodySpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySpringSystemSimulator::simulateTimestep(float timestep)
{
	
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		Vec3 gravitation(0, 0, 0);
		if (it->massInverse > 0) {
			gravitation = gravity / it->massInverse;
		}
		it->force = gravitation - damping * it->velocity;
	}
	for (vector<Spring>::iterator it = springs.begin(); it != springs.end(); ++it) {
		Spring spring = *it;
		Vec3 position1 = masspoints[spring.masspoint1].position;
		if (masspoints[spring.masspoint1].rigidBody >= 0) {
			position1 = rigidbodies[masspoints[spring.masspoint1].rigidBody].orientation.getRotMat().transformVector(position1);
			position1 += rigidbodies[masspoints[spring.masspoint1].rigidBody].position;
		}
		Vec3 position2 = masspoints[spring.masspoint2].position;
		if (masspoints[spring.masspoint2].rigidBody >= 0) {
			position2 = rigidbodies[masspoints[spring.masspoint2].rigidBody].orientation.getRotMat().transformVector(position2);
			position2 += rigidbodies[masspoints[spring.masspoint2].rigidBody].position;
		}
		double currentLength = norm(position1 - position2);
		double undirectedForce = -spring.stiffness * (currentLength - spring.initialLength);
		Vec3 force = undirectedForce * getNormalized(position1 - position2);
		masspoints[spring.masspoint1].force += force;
		masspoints[spring.masspoint2].force -= force;
	}
	for (vector<Masspoint>::iterator it = masspoints.begin(); it != masspoints.end(); ++it) {
		if (it->rigidBody >= 0) {
			it->velocity = rigidbodies[it->rigidBody].linearVelocity + cross(getAngularVelocityOfRigidBody(it->rigidBody), it->position);
			Force force;
			force.rigidbody = it->rigidBody;
			force.force = it->force;
			force.position = it->position + rigidbodies[it->rigidBody].position;
			forces.push_back(force);
		}
		else {
			it->velocity += it->force * it->massInverse * timestep;
			it->position += it->velocity * timestep;
		}
	}
	for (vector<Rigidbody>::iterator it = rigidbodies.begin(); it != rigidbodies.end(); ++it) {
		if (it->massInverse > 0) {
			it->force = gravity / it->massInverse;
		}
		else {
			it->force = Vec3(0, 0, 0);
		}
		it->torque = Vec3(0, 0, 0);
	}
	for (vector<Force>::iterator it = forces.begin(); it != forces.end(); ++it) {
		rigidbodies[it->rigidbody].force += it->force;
		rigidbodies[it->rigidbody].torque += cross(it->position - rigidbodies[it->rigidbody].position, it->force);
	}
	forces.clear();
	for (int i = 0; i < rigidbodies.size(); i++) {
		//Damping
		rigidbodies[i].torque -= rigidbodies[i].angularMomentum * damping;
		rigidbodies[i].force -= rigidbodies[i].linearVelocity * damping;
		//Integrate linear velocity
		rigidbodies[i].linearVelocity += rigidbodies[i].force * rigidbodies[i].massInverse * timestep;
		//Integrate position
		rigidbodies[i].position += rigidbodies[i].linearVelocity * timestep;
		//Integrate angular momentum
		rigidbodies[i].angularMomentum += timestep * rigidbodies[i].torque;

		//Integrate orientation
		Vec3 angularVelocity = getAngularVelocityOfRigidBody(i);
		rigidbodies[i].orientation += timestep / 2.0 * Quaternion<double>(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0) * rigidbodies[i].orientation;
		rigidbodies[i].orientation = rigidbodies[i].orientation.unit();

		collisionDetection();
	}
}

void RigidBodySpringSystemSimulator::onClick(int x, int y)
{
}

void RigidBodySpringSystemSimulator::onMouse(int x, int y)
{
}

void RigidBodySpringSystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass)
{
	Rigidbody rigidbody;
	rigidbody.position = position;
	rigidbody.orientation = Quaternion<double>(0, 0, 0, 1);
	rigidbody.massInverse = mass;
	rigidbody.size = size;
	rigidbody.linearVelocity = Vec3(0, 0, 0);
	rigidbody.angularMomentum = Vec3(0, 0, 0);
	rigidbody.inertiaTensor = calculateInertiaTensor(rigidbody.massInverse, size);
	rigidbodies.push_back(rigidbody);
}

int RigidBodySpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, double massInverse, int rigidBody) {
	Masspoint masspoint;
	masspoint.position = position;
	masspoint.velocity = velocity;
	masspoint.force = Vec3();
	masspoint.rigidBody = rigidBody;
	masspoint.massInverse = massInverse;
	masspoints.push_back(masspoint);
	return masspoints.size() - 1;
}

void RigidBodySpringSystemSimulator::addSpring(int masspoint1, int masspoint2, double initialLength, double stiffness) {
	Spring spring;
	spring.masspoint1 = masspoint1;
	spring.masspoint2 = masspoint2;
	spring.initialLength = initialLength;
	spring.stiffness = stiffness;
	springs.push_back(spring);
}

matrix4x4<double> RigidBodySpringSystemSimulator::calculateInertiaTensor(double massInverse, Vec3 size) {
	const float x = size.x;
	const float y = size.y;
	const float z = size.z;
	const float xSquared = x * x;
	const float ySquared = y * y;
	const float zSquared = z * z;

	return GamePhysics::Mat4(
		12.0f * massInverse / (ySquared + zSquared), 0.0f, 0.0f, 0.0f,
		0.0f, 12.0f * massInverse / (xSquared + zSquared), 0.0f, 0.0f,
		0.0f, 0.0f, 12.0f * massInverse / (xSquared + ySquared), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

void RigidBodySpringSystemSimulator::setOrientationOf(int i, Quat orientation)
{
	orientation = orientation.unit();
	rigidbodies[i].orientation = orientation;
}

Vec3 RigidBodySpringSystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	matrix4x4<double> rotationMatrix = rigidbodies[i].orientation.getRotMat();
	matrix4x4<double> rotationMatrixTransposed = rotationMatrix;
	rotationMatrixTransposed.transpose();
	matrix4x4<double> inertiaTensor = rotationMatrix * rigidbodies[i].inertiaTensor * rotationMatrixTransposed;
	return inertiaTensor.transformVector(rigidbodies[i].angularMomentum);
}

void RigidBodySpringSystemSimulator::collisionDetection() {
	for (int i = 0; i < rigidbodies.size(); i++) {
		for (int j = i + 1; j < rigidbodies.size(); j++) {
			Rigidbody bodyA = rigidbodies[i];
			Rigidbody bodyB = rigidbodies[j];
			//Matrix for body A
			matrix4x4<double> scaleMatrixA;
			scaleMatrixA.initScaling(bodyA.size.x, bodyA.size.y, bodyA.size.z);
			matrix4x4<double> rotationMatrixA = bodyA.orientation.getRotMat();
			matrix4x4<double> translationMatrixA;
			translationMatrixA.initTranslation(bodyA.position.x, bodyA.position.y, bodyA.position.z);
			matrix4x4<double> objToWorldMatrixA = rotationMatrixA * translationMatrixA;
			objToWorldMatrixA = scaleMatrixA * objToWorldMatrixA;
			//Matrix for body B
			matrix4x4<double> scaleMatrixB;
			scaleMatrixB.initScaling(bodyB.size.x, bodyB.size.y, bodyB.size.z);
			matrix4x4<double> rotationMatrixB = bodyB.orientation.getRotMat();
			matrix4x4<double> translationMatrixB;
			translationMatrixB.initTranslation(bodyB.position.x, bodyB.position.y, bodyB.position.z);
			matrix4x4<double> objToWorldMatrixB = rotationMatrixB * translationMatrixB;
			objToWorldMatrixB = scaleMatrixB * objToWorldMatrixB;

			CollisionInfo collision = checkCollisionSAT(objToWorldMatrixA, objToWorldMatrixB);

			//Check if collision has accured
			if (!collision.isValid) {
				continue;
			}

			//Calculate inertia tensors and angular velocities
			matrix4x4<double> rotationMatrixATransposed = bodyA.orientation.getRotMat();
			rotationMatrixATransposed.transpose();
			matrix4x4<double> inertiaTensorA = rotationMatrixA * bodyA.inertiaTensor * rotationMatrixATransposed;
			Vec3 angularVelocityA = inertiaTensorA.transformVector(bodyA.angularMomentum);

			matrix4x4<double> rotationMatrixBTransposed = bodyB.orientation.getRotMat();
			rotationMatrixBTransposed.transpose();
			matrix4x4<double> inertiaTensorB = rotationMatrixB * bodyB.inertiaTensor * rotationMatrixBTransposed;
			Vec3 angularVelocityB = inertiaTensorB.transformVector(bodyB.angularMomentum);

			//Calculate relative velocity
			collision.collisionPointWorld;

			Vec3 collisionPointA = collision.collisionPointWorld - bodyA.position;
			Vec3 collisionPointB = collision.collisionPointWorld - bodyB.position;
			Vec3 velocityA = bodyA.linearVelocity + cross(angularVelocityA, collisionPointA);
			Vec3 velocityB = bodyB.linearVelocity + cross(angularVelocityB, collisionPointB);
			Vec3 relativeVelocity = velocityA - velocityB;

			//Check if bodies are seperating
			if (dot(relativeVelocity, collision.normalWorld) > 0.0) {
				continue;
			}

			//Calculate impulse
			double a = -(1.0 + BOUNCINESS) * dot(relativeVelocity, collision.normalWorld);
			double b = 1.0 * bodyA.massInverse + 1.0 * bodyB.massInverse;
			Vec3 c = cross(inertiaTensorA.transformVector(cross(collisionPointA, collision.normalWorld)), collisionPointA);
			Vec3 d = cross(inertiaTensorB.transformVector(cross(collisionPointB, collision.normalWorld)), collisionPointB);
			double e = dot(c + d, collision.normalWorld);
			double impulse = a / (b + e);

			//Uncollide objects
			//rigidbodies[i].position += collision.depth * collision.normalWorld;
			//rigidbodies[j].position -= collision.depth * collision.normalWorld;

			//Apply impulse
			rigidbodies[i].linearVelocity += impulse * collision.normalWorld * bodyA.massInverse;
			rigidbodies[j].linearVelocity -= impulse * collision.normalWorld * bodyB.massInverse;

			rigidbodies[i].angularMomentum += cross(collisionPointA, impulse * collision.normalWorld);
			rigidbodies[j].angularMomentum -= cross(collisionPointB, impulse * collision.normalWorld);
		}
	}
}