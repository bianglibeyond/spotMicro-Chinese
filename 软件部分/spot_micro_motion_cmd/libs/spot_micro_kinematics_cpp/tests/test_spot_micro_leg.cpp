#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "spot_micro_kinematics/spot_micro_leg.h"

using namespace std;
using namespace Eigen;
using namespace smk;

TEST(setFootPos, local_coordinates)
{
  
  // Create a leg object with a homogeneous transform that is 
  // far translated. Call method to set a reachable point within the legs own
  // coordinate system (but which would not be a point reachable if it were in a
  // global coordinate system). 
  // Verify foot placed at that point.
  float d2r = M_PI/180.0f; 

  Point desired_point = {0.02f, -0.2f, 0.01f};
  JointAngles joint_angles = {0.0f, 0.0f, 0.0f}; 
  LinkLengths link_lengths = {0.1f, 0.4f, 0.4f};

  // Sample body ht that is far translated
  Matrix4f body_ht = homogTransXyz(10.0f, 20.0f, 30.0f) *
                     homogRotXyz(5*d2r, 5*d2r, 5*d2r); 
 
  // Make the leg start ht, put in zero boyd width and length 
  Matrix4f ht_leg_start = htLegRightFront(body_ht, 0.0f, 0.0f); 
  
  // Create leg object 
  SpotMicroLeg sml = SpotMicroLeg(joint_angles, link_lengths, true);

  // Call method to set foot position in local coordinates
  sml.setFootPosLocalCoordinates(desired_point);
  
  // Get foot position in global coordinates
  Point foot_point_global = sml.getFootPosGlobalCoordinates(ht_leg_start);

  // Create a homogeneous vector with that point
  Vector4f test_point_vec(foot_point_global.x,
                          foot_point_global.y,
                          foot_point_global.z,
                          1.0f);

  // Express that position in leg's local coordinate frame
  auto leg_local_foot_pt = homogInverse(ht_leg_start) * test_point_vec;

  // Test equality to 0.0001
  EXPECT_NEAR(desired_point.x, leg_local_foot_pt(0), 0.0001f);
  EXPECT_NEAR(desired_point.y, leg_local_foot_pt(1), 0.0001f);
  EXPECT_NEAR(desired_point.z, leg_local_foot_pt(2), 0.0001f);
}


TEST(setFootPos, global_coordinates)
{

  // Create a leg object with some non zero starting homogeneous trasnform, and
  // leg links. Call method to set a reachable point. Verify foot placed at that
  // point.
  float d2r = M_PI/180.0f; 

  Point desired_point = {0.02f, 0.01f, 0.01f};
  JointAngles joint_angles = {0.0f, 0.0f, 0.0f}; 
  LinkLengths link_lengths = {0.1f, 0.4f, 0.4f};

  // Sample body ht that is translated and rotated slightly
  Matrix4f body_ht = homogTransXyz(0.05f, 0.14f, 0.05f) *
                     homogRotXyz(5*d2r, 5*d2r, 5*d2r); 
 
  // Make the leg start ht, put in zero boyd width and length 
  Matrix4f ht_leg_start = htLegRightFront(body_ht, 0.0f, 0.0f); 
  
  // Create leg object 
  SpotMicroLeg sml = SpotMicroLeg(joint_angles, link_lengths, true);

  // Call method to set foot position in global coordinates
  sml.setFootPosGlobalCoordinates(desired_point, ht_leg_start);

  // Get foot position
  Point test_point = sml.getFootPosGlobalCoordinates(ht_leg_start);

  // Test equality to 0.0001
  EXPECT_NEAR(desired_point.x, test_point.x, 0.0001f);
  EXPECT_NEAR(desired_point.y, test_point.y, 0.0001f);
  EXPECT_NEAR(desired_point.z, test_point.z, 0.0001f);
}



