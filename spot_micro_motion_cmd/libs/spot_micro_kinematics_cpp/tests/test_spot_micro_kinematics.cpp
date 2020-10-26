#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "spot_micro_kinematics/spot_micro_kinematics.h"

using namespace std;
using namespace Eigen;
using namespace smk;

TEST(testSetJointAngles, basic_test)
{
  
  float d2r = M_PI/180.0f; 
  
  // Create a spot micro kinematics object, set the joint angles, make sure they
  // are set for the legs
  SpotMicroConfig smc = {0.1, 0.2, 0.3, 0.4, 0.5};

  SpotMicroKinematics sm = SpotMicroKinematics(0.0f, 0.0f, 0.0f, smc);

  JointAngles desired = {10*d2r, 20*d2r, -15*d2r};

  LegsJointAngles four_legs_desired_angs = {desired, desired, desired, desired};

  sm.setLegJointAngles(four_legs_desired_angs);

  LegsJointAngles test_legs_joint_angs = sm.getLegsJointAngles();

  // Test equality
  EXPECT_EQ(four_legs_desired_angs.right_back.ang1, test_legs_joint_angs.right_back.ang1); 
  EXPECT_EQ(four_legs_desired_angs.right_back.ang2, test_legs_joint_angs.right_back.ang2); 
  EXPECT_EQ(four_legs_desired_angs.right_back.ang3, test_legs_joint_angs.right_back.ang3); 
  
  EXPECT_EQ(four_legs_desired_angs.right_front.ang1, test_legs_joint_angs.right_front.ang1); 
  EXPECT_EQ(four_legs_desired_angs.right_front.ang2, test_legs_joint_angs.right_front.ang2); 
  EXPECT_EQ(four_legs_desired_angs.right_front.ang3, test_legs_joint_angs.right_front.ang3); 

  EXPECT_EQ(four_legs_desired_angs.left_front.ang1, test_legs_joint_angs.left_front.ang1); 
  EXPECT_EQ(four_legs_desired_angs.left_front.ang2, test_legs_joint_angs.left_front.ang2); 
  EXPECT_EQ(four_legs_desired_angs.left_front.ang3, test_legs_joint_angs.left_front.ang3); 

  EXPECT_EQ(four_legs_desired_angs.left_back.ang1, test_legs_joint_angs.left_back.ang1); 
  EXPECT_EQ(four_legs_desired_angs.left_back.ang2, test_legs_joint_angs.left_back.ang2); 
  EXPECT_EQ(four_legs_desired_angs.left_back.ang3, test_legs_joint_angs.left_back.ang3); 
}


TEST(testSetFootPos, basic_test)
{
  
  float d2r = M_PI/180.0f; 
  
  // Create a spot micro kinematics object, set reachable foot positions given
  // the leg and body geometry
  SpotMicroConfig smc = {0.1, 0.4, 0.4, 0.4, 0.5};
  SpotMicroKinematics sm = SpotMicroKinematics(0.05f, 0.03f, 0.01f, smc);

  LegsFootPos four_legs_desired_pos = {.right_back = {-0.22, -0.22, 0.32},
                                           .right_front = {0.22, -0.22, 0.22},
                                           .left_front = {0.22, -0.22, -0.22},
                                           .left_back = {-0.22, -0.22, -0.22}};

  sm.setFeetPosGlobalCoordinates(four_legs_desired_pos);

  LegsFootPos test_legs_foot_pos = sm.getLegsFootPos();

  // Test value nearness to 0.0001f
  float tol = 0.0001f; 
  EXPECT_NEAR(four_legs_desired_pos.right_back.x, test_legs_foot_pos.right_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.y, test_legs_foot_pos.right_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.z, test_legs_foot_pos.right_back.z, tol); 
  
  EXPECT_NEAR(four_legs_desired_pos.right_front.x, test_legs_foot_pos.right_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.y, test_legs_foot_pos.right_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.z, test_legs_foot_pos.right_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_front.x, test_legs_foot_pos.left_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.y, test_legs_foot_pos.left_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.z, test_legs_foot_pos.left_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_back.x, test_legs_foot_pos.left_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.y, test_legs_foot_pos.left_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.z, test_legs_foot_pos.left_back.z, tol); 
}


TEST(testSetBodyAngle, basic_test)
{
  
  float d2r = M_PI/180.0f; 
  
  // Create a spot micro kinematics object, set reachable foot positions given
  // the leg and body geometry.
  //
  // Get the joint angles.
  //
  // Set body angles
  //
  // Get the feet position and joint angles
  //
  // Verify the feet position still match after all operations, but that the
  // joint angles are different (verifying the legs actually changes but the
  // feet are in the same spot)
  

  SpotMicroConfig smc = {0.1, 0.4, 0.4, 0.4, 0.5};
  SpotMicroKinematics sm = SpotMicroKinematics(0.05f, 0.03f, 0.01f, smc);

  LegsFootPos four_legs_desired_pos = {.right_back = {-0.22, -0.22, 0.32},
                                           .right_front = {0.22, -0.22, 0.22},
                                           .left_front = {0.22, -0.22, -0.22},
                                           .left_back = {-0.22, -0.22, -0.22}};

  sm.setFeetPosGlobalCoordinates(four_legs_desired_pos);

  LegsJointAngles truth_joint_angles = sm.getLegsJointAngles();

  sm.setBodyAngles(5.0*d2r, 3.0*d2r, -2.0*d2r);

  LegsFootPos test_legs_foot_pos = sm.getLegsFootPos();
  LegsJointAngles test_legs_joint_angs = sm.getLegsJointAngles();

  // Test value nearness to 0.0001f
  float tol = 0.0001f; 
  EXPECT_NEAR(four_legs_desired_pos.right_back.x, test_legs_foot_pos.right_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.y, test_legs_foot_pos.right_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.z, test_legs_foot_pos.right_back.z, tol); 
  
  EXPECT_NEAR(four_legs_desired_pos.right_front.x, test_legs_foot_pos.right_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.y, test_legs_foot_pos.right_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.z, test_legs_foot_pos.right_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_front.x, test_legs_foot_pos.left_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.y, test_legs_foot_pos.left_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.z, test_legs_foot_pos.left_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_back.x, test_legs_foot_pos.left_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.y, test_legs_foot_pos.left_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.z, test_legs_foot_pos.left_back.z, tol); 

  // Test that joint angles are different
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang1 -
                  test_legs_joint_angs.right_back.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang2 -
                  test_legs_joint_angs.right_back.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang3 -
                  test_legs_joint_angs.right_back.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang1 -
                  test_legs_joint_angs.right_front.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang2 -
                  test_legs_joint_angs.right_front.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang3 -
                  test_legs_joint_angs.right_front.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang1 -
                  test_legs_joint_angs.left_front.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang2 -
                  test_legs_joint_angs.left_front.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang3 -
                  test_legs_joint_angs.left_front.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang1 -
                  test_legs_joint_angs.left_back.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang2 -
                  test_legs_joint_angs.left_back.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang3 -
                  test_legs_joint_angs.left_back.ang3) > tol); 
}


TEST(testSetBodyPos, basic_test)
{
  
  float d2r = M_PI/180.0f; 
  
  // Create a spot micro kinematics object, set reachable foot positions given
  // the leg and body geometry.
  //
  // Get the joint angles.
  //
  // Set body position
  //
  // Get the feet position and joint angles
  //
  // Verify the feet position still match after all operations, but that the
  // joint angles are different (verifying the legs actually changes but the
  // feet are in the same spot)
  

  SpotMicroConfig smc = {0.1, 0.4, 0.4, 0.4, 0.5};
  SpotMicroKinematics sm = SpotMicroKinematics(0.05f, 0.03f, 0.01f, smc);

  LegsFootPos four_legs_desired_pos = {.right_back = {-0.22, -0.22, 0.32},
                                           .right_front = {0.22, -0.22, 0.22},
                                           .left_front = {0.22, -0.22, -0.22},
                                           .left_back = {-0.22, -0.22, -0.22}};

  sm.setFeetPosGlobalCoordinates(four_legs_desired_pos);

  LegsJointAngles truth_joint_angles = sm.getLegsJointAngles();

  sm.setBodyPosition(0.052f, -0.053f, 0.05f);

  LegsFootPos test_legs_foot_pos = sm.getLegsFootPos();
  LegsJointAngles test_legs_joint_angs = sm.getLegsJointAngles();

  // Test value nearness to 0.0001f
  float tol = 0.0001f; 
  EXPECT_NEAR(four_legs_desired_pos.right_back.x, test_legs_foot_pos.right_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.y, test_legs_foot_pos.right_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_back.z, test_legs_foot_pos.right_back.z, tol); 
  
  EXPECT_NEAR(four_legs_desired_pos.right_front.x, test_legs_foot_pos.right_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.y, test_legs_foot_pos.right_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.right_front.z, test_legs_foot_pos.right_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_front.x, test_legs_foot_pos.left_front.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.y, test_legs_foot_pos.left_front.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_front.z, test_legs_foot_pos.left_front.z, tol); 

  EXPECT_NEAR(four_legs_desired_pos.left_back.x, test_legs_foot_pos.left_back.x, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.y, test_legs_foot_pos.left_back.y, tol); 
  EXPECT_NEAR(four_legs_desired_pos.left_back.z, test_legs_foot_pos.left_back.z, tol); 

  // Test that joint angles are different
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang1 -
                  test_legs_joint_angs.right_back.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang2 -
                  test_legs_joint_angs.right_back.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_back.ang3 -
                  test_legs_joint_angs.right_back.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang1 -
                  test_legs_joint_angs.right_front.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang2 -
                  test_legs_joint_angs.right_front.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.right_front.ang3 -
                  test_legs_joint_angs.right_front.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang1 -
                  test_legs_joint_angs.left_front.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang2 -
                  test_legs_joint_angs.left_front.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_front.ang3 -
                  test_legs_joint_angs.left_front.ang3) > tol); 

  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang1 -
                  test_legs_joint_angs.left_back.ang1) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang2 -
                  test_legs_joint_angs.left_back.ang2) > tol); 
  EXPECT_TRUE(abs(truth_joint_angles.left_back.ang3 -
                  test_legs_joint_angs.left_back.ang3) > tol); 
}


TEST(testReturnBodyState, basic_test)
{
  float d2r = M_PI/180.0; 
  float truth_x = 0.1; float truth_y = 0.12; float truth_z = 0.13;
  float truth_phi = 0.1; float truth_theta = 0.05f; float truth_psi = -0.02f;
  
  // Create a spot micro kinematics object
  SpotMicroConfig smc = {0.1, 0.2, 0.3, 0.4, 0.5};

  SpotMicroKinematics sm = SpotMicroKinematics(truth_x, truth_y, truth_z, smc);

  JointAngles desired = {10*d2r, 20*d2r, -15*d2r};

  LegsJointAngles four_legs_desired_angs = {desired, desired, desired, desired};

  sm.setLegJointAngles(four_legs_desired_angs);

  sm.setBodyAngles(truth_phi, truth_theta, truth_psi);

  BodyState test_body_state = sm.getBodyState();

  // Test equality
  EXPECT_FLOAT_EQ(truth_x, test_body_state.xyz_pos.x); 
  EXPECT_FLOAT_EQ(truth_y, test_body_state.xyz_pos.y); 
  EXPECT_FLOAT_EQ(truth_z, test_body_state.xyz_pos.z); 

  EXPECT_FLOAT_EQ(truth_phi, test_body_state.euler_angs.phi); 
  EXPECT_FLOAT_EQ(truth_theta, test_body_state.euler_angs.theta); 
  EXPECT_FLOAT_EQ(truth_psi, test_body_state.euler_angs.psi); 
}
