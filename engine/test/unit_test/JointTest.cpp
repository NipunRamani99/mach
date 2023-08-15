#include <gtest/gtest.h>
#include "include/Collision.hpp"
#include "include/RigidBody.hpp"
#include "include/RevoluteJoint.hpp"
#include "include/MouseJoint.hpp"
TEST(JointTest, RevoluteJointTest) {
	BoxRigidBody bodyA = BoxRigidBody({ 0.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, FLT_MAX, 0.0f, { 1.0f,1.0f,1.0f }, false);
	BoxRigidBody bodyB = BoxRigidBody({ 5.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, 1.0f, 0.0f, { 1.0f,1.0f,1.0f }, false);
	bodyA.linear_velocity = { -1.0f, 0.0f };
	bodyA.angular_velocity = 0.0f;
	bodyB.linear_velocity = { 1.0f,0.0f };
	bodyB.angular_velocity = 0.0f;
	bodyA.inv_mass = 0.0f;
	bodyA.inv_inertia = 0.0f;
	bodyA.inertia = FLT_MAX;

	RevoluteJoint joint(&bodyA, &bodyB, bodyA.position);
	float dt = 1.0f/60.0f;
	joint.initialize(dt);
	joint.applyImpulse(dt);

	ASSERT_FLOAT_EQ(joint.P.x, -2.0f);
	ASSERT_FLOAT_EQ(joint.P.y, 0.0f);

	ASSERT_FLOAT_EQ(bodyA.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyA.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyA.angular_velocity, 0.0f);

	ASSERT_FLOAT_EQ(bodyB.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyB.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyB.angular_velocity, 0.0f);
}


TEST(JointTest, MouseJointTest) {
	BoxRigidBody bodyA = BoxRigidBody({ 0.0f,0.0f }, { 100.0f, 100.0f }, 0.0f, 1.0f, 0.30f, { 1.0f,1.0f,1.0f }, false);
	bodyA.linear_velocity = { 1.0f,0.0f };
	bodyA.angular_velocity = 0.0f;

	MouseJoint joint(&bodyA, { 0.0f, 40.0f }, { 0.0f, 40.0f });
	ASSERT_FLOAT_EQ(joint.localCenter.x, 0.0f);
	ASSERT_FLOAT_EQ(joint.localCenter.y, 40.0f);

	float dt = 1.0f / 60.0f;
	joint.initialize(dt);

	ASSERT_FLOAT_EQ(joint.mass[0][0], 0.361591f);
	ASSERT_FLOAT_EQ(joint.mass[0][1], 0.0f);
	ASSERT_FLOAT_EQ(joint.mass[1][0], 0.0f);
	ASSERT_FLOAT_EQ(joint.mass[1][1], 0.553846f);


	ASSERT_FLOAT_EQ(joint.C.x, 0.0f);
	ASSERT_FLOAT_EQ(joint.C.y, 0.0f);

	joint.applyImpulse(dt);

	ASSERT_FLOAT_EQ(joint.impulse.x, -0.36159101f);
	ASSERT_FLOAT_EQ(joint.impulse.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyA.linear_velocity.x, 0.638409f);
	ASSERT_FLOAT_EQ(bodyA.linear_velocity.y, 0.00f);
	ASSERT_FLOAT_EQ(bodyA.angular_velocity, 0.0086781848);
}

