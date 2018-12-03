#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	
}

void RigidBodySystemSimulator::reset()
{
	rigidbodies.clear();
	forces.clear();
	gravity = 0;

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
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
}

void RigidBodySystemSimulator::demo1() {
	simulateTimestep(0.1);
}

//Noch ändern
void RigidBodySystemSimulator::printSolution() {
	for (int i = 0; i < rigidbodies.size(); i++) {
		cout << "Masspoint " << i << ":\n";
		cout << "  x = " << rigidbodies[i].position.x << "\n";
		cout << "  y = " << rigidbodies[i].position.y << "\n";
		cout << "  z = " << rigidbodies[i].position.z << "\n";
	}
}

void RigidBodySystemSimulator::demo2() {
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	matrix4x4<double> rotationMatrix = matrix4x4<double>();
	rotationMatrix.initRotationZ(90);
	Quaternion<double> rotationQuaternion = Quaternion<double>(rotationMatrix);
	setOrientationOf(0, rotationQuaternion);

	Force force;
	force.rigidbody = 0;
	force.position = Vec3(0.3, 0.5, 0.25);
	force.force = Vec3(100, 100, 0);
	forces.push_back(force);
}

void RigidBodySystemSimulator::demo3() {
	addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.5, 0.5), 2);
	addRigidBody(Vec3(1, 0, 0.1), Vec3(0.5, 0.5, 0.5), 3);
	setVelocityOf(0, Vec3(1, 0, 0));
	setVelocityOf(1, Vec3(-0.5, 0, 0));
	matrix4x4<double> rotation = matrix4x4<double>(0);
	rotation.initRotationXYZ(40, 40, 0);
	setOrientationOf(1, Quaternion<double>(rotation));
}
//Fix this collision
void RigidBodySystemSimulator::demo4() {
	addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.5, 0.5), 2);
	addRigidBody(Vec3(1, 0, 0.1), Vec3(0.5, 0.5, 0.5), 3);
	addRigidBody(Vec3(0, 1, 0), Vec3(0.5, 0.5, 0.5), 4);
	addRigidBody(Vec3(-0.1, -1, 0), Vec3(0.5, 0.5, 0.5), 1);
	setVelocityOf(0, Vec3(1, 0, 0));
	setVelocityOf(1, Vec3(-0.5, 0, 0));
	setVelocityOf(2, Vec3(0, -0.5, 0));
	setVelocityOf(3, Vec3(0, 1, 0));
	matrix4x4<double> rotation = matrix4x4<double>(0);
	rotation.initRotationXYZ(40, 40, 0);
	setOrientationOf(0, Quaternion<double>(rotation));
	setOrientationOf(1, Quaternion<double>(rotation));
	setOrientationOf(2, Quaternion<double>(rotation));
	setOrientationOf(3, Quaternion<double>(rotation));
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	switch (testCase) {
	case 0:
		demo1();
		break;
	case 1:
		demo2();
		break;
	case 2:
		demo3();
		break;
	case 3:
		demo4();
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::collisionDetection() {
	for (int i = 0; i < rigidbodies.size(); i++) {
		for (int j = i + 1; j < rigidbodies.size(); j++) {
			Rigidbody bodyA = rigidbodies[i];
			Rigidbody bodyB = rigidbodies[j];
			//Matrix for body A
			matrix4x4<double> scaleMatrix;
			scaleMatrix.initScaling(bodyA.size.x, bodyA.size.y, bodyA.size.z);
			matrix4x4<double> rotationMatrix = bodyA.orientation.getRotMat();
			matrix4x4<double> translationMatrix;
			translationMatrix.initTranslation(bodyA.position.x, bodyA.position.y, bodyA.position.z);
			matrix4x4<double> objToWorldMatrixA = scaleMatrix * rotationMatrix * translationMatrix;
			//Matrix for body B
			scaleMatrix.initScaling(bodyB.size.x, bodyB.size.y, bodyB.size.z);
			rotationMatrix = bodyB.orientation.getRotMat();
			translationMatrix.initTranslation(bodyB.position.x, bodyB.position.y, bodyB.position.z);
			matrix4x4<double> objToWorldMatrixB = scaleMatrix * rotationMatrix * translationMatrix;
			CollisionInfo collision = checkCollisionSAT(objToWorldMatrixA, objToWorldMatrixB);

			//Check if collision has accured
			if (!collision.isValid) {
				break;
			}

			//Calculate angular velocities
			rotationMatrix = bodyA.orientation.getRotMat();
			matrix4x4<double> rotationMatrixTransposed = bodyA.orientation.getRotMat();
			rotationMatrixTransposed.transpose();
			matrix4x4<double> inertiaTensorA = rotationMatrixTransposed * bodyA.inertiaTensor * rotationMatrix;
			Vec3 angularVelocityA = inertiaTensorA.transformVector(bodyA.angularMomentum);

			rotationMatrix = bodyB.orientation.getRotMat();
			rotationMatrixTransposed = bodyB.orientation.getRotMat();
			rotationMatrixTransposed.transpose();
			matrix4x4<double> inertiaTensorB = rotationMatrixTransposed * bodyB.inertiaTensor * rotationMatrix;
			Vec3 angularVelocityB = inertiaTensorB.transformVector(bodyA.angularMomentum);

			//Calculate relative velocity
			Vec3 collisionPointA = collision.collisionPointWorld - bodyA.position;
			Vec3 collisionPointB = collision.collisionPointWorld - bodyB.position;
			Vec3 velocityA = bodyA.linearVelocity + cross(collisionPointA, angularVelocityA);
			Vec3 velocityB = bodyB.linearVelocity + cross(collisionPointB, angularVelocityB);
			Vec3 relativeVelocity = velocityA - velocityB;
			
			//Check if bodies are seperating
			if (dot(relativeVelocity, collision.normalWorld) > 0) {
				break;
			}

			//Calculate impulse
			double a = (dot(-(1 + BOUNCINESS) * relativeVelocity, collision.normalWorld));
			double b = 1.0 / bodyA.mass + 1.0 / bodyB.mass;
			Vec3 c = inertiaTensorA.transformVector(cross(cross(collisionPointA, collision.normalWorld), collisionPointA));
			Vec3 d = inertiaTensorB.transformVector(cross(cross(collisionPointB, collision.normalWorld), collisionPointB));
			double e = dot(c + d, collision.normalWorld);
			double impulse = a / (b + e);

			//Uncollide objects
			//rigidbodies[i].position += collision.depth * collision.normalWorld;
			//rigidbodies[j].position -= collision.depth * collision.normalWorld;
			//Apply impulse
			rigidbodies[i].linearVelocity += impulse * collision.normalWorld / bodyA.mass;
			rigidbodies[j].linearVelocity -= impulse * collision.normalWorld / bodyB.mass;

			rigidbodies[i].angularMomentum += cross(collisionPointA, impulse * collision.normalWorld);
			rigidbodies[j].angularMomentum -= cross(collisionPointB, impulse * collision.normalWorld);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timestep)
{
	for (vector<Rigidbody>::iterator it = rigidbodies.begin(); it != rigidbodies.end(); ++it) {
		it->force = gravity * it->mass;
		it->torque = Vec3(0, 0, 0);
	}
	for (vector<Force>::iterator it = forces.begin(); it != forces.end(); ++it) {
		rigidbodies[it->rigidbody].force += it->force;
		rigidbodies[it->rigidbody].torque += cross(it->position - rigidbodies[it->rigidbody].position, it->force);
	}
	forces.clear();
	for (vector<Rigidbody>::iterator it = rigidbodies.begin(); it != rigidbodies.end(); ++it) {
		//Integrate position
		it->position += it->linearVelocity * timestep;
		//Integrate linear velocity
		it->linearVelocity += it->force / it->mass * timestep;

		//Integrate orientation
		matrix4x4<double> rotationMatrix = it->orientation.getRotMat();
		matrix4x4<double> rotationMatrixTransposed = it->orientation.getRotMat();
		rotationMatrixTransposed.transpose();
		matrix4x4<double> inertiaTensor = rotationMatrixTransposed * it->inertiaTensor * rotationMatrix;
		Vec3 angularVelocity = inertiaTensor.transformVector(it->angularMomentum);
		it->orientation = it->orientation + timestep / 2.0 * Quaternion<double>(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0) * it->orientation;
		//it->orientation /= it->orientation.norm();
		it->orientation = it->orientation.unit();
		//Integrate angular momentum
		it->angularMomentum += timestep * it->torque;

		collisionDetection();
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidbodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidbodies[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	matrix4x4<double> rotationMatrix = rigidbodies[i].orientation.getRotMat();
	matrix4x4<double> rotationMatrixTransposed = rotationMatrix;
	rotationMatrixTransposed.transpose();
	matrix4x4<double> inertiaTensor = rotationMatrixTransposed * rigidbodies[i].inertiaTensor * rotationMatrix;
	return inertiaTensor.transformVector(rigidbodies[i].angularMomentum);
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	Force f;
	f.rigidbody = i;
	f.position = loc;
	f.force = force;
	forces.push_back(f);
}

matrix4x4<double> RigidBodySystemSimulator::calculateInertiaTensor(double mass, Vec3 size) {
	double width = size.x;
	double height = size.y;
	double depth = size.z;
	double iWidth = 1.0 / 12.0 * mass * (height * height + depth * depth);
	double iHeight = 1.0 / 12.0 * mass * (width * width + depth * depth);
	double iDepth = 1.0 / 12.0 * mass * (width * width + height * height);
	matrix4x4<double> inertiaTensor = matrix4x4<double>(iWidth, 0, 0, 0,
														0, iHeight, 0, 0,
														0, 0, iDepth, 0,
														0, 0, 0, 1);
	return inertiaTensor.inverse();
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	Rigidbody rigidbody;
	rigidbody.position = position;
	rigidbody.orientation = Quaternion<double>(0, 0, 0, 1);
	rigidbody.mass = mass;
	rigidbody.size = size;
	rigidbody.linearVelocity = Vec3(0, 0, 0);
	rigidbody.angularMomentum = Vec3(0, 0, 0);
	rigidbody.inertiaTensor = calculateInertiaTensor(rigidbody.mass, size);
	rigidbodies.push_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	orientation = orientation.unit();
	rigidbodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodies[i].linearVelocity = velocity;
}
