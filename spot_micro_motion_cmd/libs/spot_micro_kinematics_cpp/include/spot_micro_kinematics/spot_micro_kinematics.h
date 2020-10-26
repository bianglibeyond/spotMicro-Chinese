#pragma once

#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"


namespace smk {

// Struct to hold joint angles for four legs
struct LegsJointAngles {
  JointAngles right_back;
  JointAngles right_front;
  JointAngles left_front;
  JointAngles left_back;
};

struct LegsFootPos {
  Point right_back;
  Point right_front;
  Point left_front;
  Point left_back;
};

// Struct to hold various configuration values of a spot micro robot frame
struct SpotMicroConfig {
  float hip_link_length;
  float upper_leg_link_length;
  float lower_leg_link_length;
  float body_width;
  float body_length;
};

// Struct to hold euler angles
struct EulerAngs {
  float phi;
  float theta;
  float psi;
};

// Struct to hold body position and euler angles
struct BodyState {
  EulerAngs euler_angs;
  Point xyz_pos;
  LegsFootPos leg_feet_pos;
};

class SpotMicroKinematics {

 public:
  // Constructor, sets up a spot micro kinematics object
  SpotMicroKinematics(float x, float y, float z, const SpotMicroConfig& smc);

  // Default Constructor
  SpotMicroKinematics() = default;

  // Returns the body center homogenous transformation matrix
  Eigen::Matrix4f getBodyHt();

  // Sets the joint angles for the legs in the robot
  void setLegJointAngles(const LegsJointAngles& four_legs_joint_angs);

  // Sets the foot for each leg to the commanded position in a global coordinate
  // system
  void setFeetPosGlobalCoordinates(const LegsFootPos& four_legs_foot_pos);

  // Sets a body rotation without translating the body or moving the feet
  void setBodyAngles(float phi, float theta, float psi);

  // Sets a body position without rotation of the body, or moving the feet
  void setBodyPosition(float x, float y, float z); 

  // Sets the body state: feet position, body position and angles
  void setBodyState(const BodyState& body_state);

  // Returns the joint angles of the four legs
  LegsJointAngles getLegsJointAngles();

  // Return the position of the four feet
  LegsFootPos getLegsFootPos();

  // Returns body state: feet positio, body position and angles
  BodyState getBodyState();
 private:

  SpotMicroConfig smc_; // Spot micro config struct
 
  // x, y, z position of body center in global coordinate system 
  float x_;
  float y_;
  float z_;
 
  // Euler angles of body in global coordinate system
  float phi_;
  float theta_;
  float psi_;
 
  // Leg objects of the robot 
  SpotMicroLeg right_back_leg_;
  SpotMicroLeg right_front_leg_;
  SpotMicroLeg left_front_leg_;
  SpotMicroLeg left_back_leg_;

};




}








