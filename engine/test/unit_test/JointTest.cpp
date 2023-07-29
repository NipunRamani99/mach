#include <gtest/gtest.h>
#include "include/Collision.hpp"
#include "include/RigidBody.hpp"
#include "include/Joint.hpp"
#include "include/DistanceJoint.hpp"

TEST(JointTest, PointToPointTest) {
	BoxRigidBody bodyA = BoxRigidBody({ 0.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, FLT_MAX, 0.0f, { 1.0f,1.0f,1.0f }, false);
	BoxRigidBody bodyB = BoxRigidBody({ 5.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, 1.0f, 0.0f, { 1.0f,1.0f,1.0f }, false);
	bodyA.linear_velocity = { -1.0f, 0.0f };
	bodyA.angular_velocity = 0.0f;
	bodyB.linear_velocity = { 1.0f,0.0f };
	bodyB.angular_velocity = 0.0f;
	bodyA.inv_mass = 0.0f;
	bodyA.inv_inertia = 0.0f;
	bodyA.inertia = FLT_MAX;

	Joint joint(&bodyA, &bodyB, bodyA.position);
	float inv_dt = 60.0f;
	joint.preStep(inv_dt);
	joint.applyImpulse();

	ASSERT_FLOAT_EQ(joint.P.x, -2.0f);
	ASSERT_FLOAT_EQ(joint.P.y, 0.0f);

	ASSERT_FLOAT_EQ(bodyA.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyA.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyA.angular_velocity, 0.0f);

	ASSERT_FLOAT_EQ(bodyB.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyB.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyB.angular_velocity, 0.0f);
}

TEST(JointTest, DistanceJoint) {
	BoxRigidBody bodyA = BoxRigidBody({ 0.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, FLT_MAX, 0.0f, { 1.0f,1.0f,1.0f }, false);
	BoxRigidBody bodyB = BoxRigidBody({ 5.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, 1.0f, 0.0f, { 1.0f,1.0f,1.0f }, false);
	bodyA.linear_velocity = { -1.0f, 0.0f };
	bodyA.angular_velocity = 0.0f;
	bodyB.linear_velocity = { 1.0f,0.0f };
	bodyB.angular_velocity = 0.0f;
	bodyA.inv_mass = 0.0f;
	bodyA.inv_inertia = 0.0f;
	bodyA.inertia = FLT_MAX;
	
	//DistanceJoint joint((RigidBody*)&bodyA, (RigidBody*)&bodyB, anchorA, anchorB, );
	//float inv_dt = 60.0f;
	//joint.preStep(inv_dt);
	//joint.applyImpulse();

	//ASSERT_FLOAT_EQ(joint.P.x, -2.0f);
	//ASSERT_FLOAT_EQ(joint.P.y, 0.0f);

	//ASSERT_FLOAT_EQ(bodyA.linear_velocity.x, -1.0f);
	//ASSERT_FLOAT_EQ(bodyA.linear_velocity.y, 0.0f);
	//ASSERT_FLOAT_EQ(bodyA.angular_velocity, 0.0f);

	//ASSERT_FLOAT_EQ(bodyB.linear_velocity.x, -1.0f);
	//ASSERT_FLOAT_EQ(bodyB.linear_velocity.y, 0.0f);
	//ASSERT_FLOAT_EQ(bodyB.angular_velocity, 0.0f);
}