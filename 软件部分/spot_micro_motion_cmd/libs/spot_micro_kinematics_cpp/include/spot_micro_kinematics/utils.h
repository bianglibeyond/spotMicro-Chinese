#pragma once // So header is only included once

#include <eigen3/Eigen/Geometry>

namespace smk
{

struct Point {
  float x;
  float y;
  float z;
};

struct LinkLengths {
  float l1;
  float l2;
  float l3;
};

struct JointAngles {
  float ang1;
  float ang2;
  float ang3;
};

// Returns a 4x4 Matrix that represents a homogeneous rotation matrix
// in the order x, y, z. Input angles are in units radians.
// A convenience wrapper around Eigen::Transform build up
Eigen::Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang);

// Returns a 4x4 matrix that represents a homogeneous translation matrix.
// Input distances of x, y, z
// A convenience wrapper around Eigen::Transform
Eigen::Matrix4f homogTransXyz(float x, float y, float z);


// Returns the inverse of the inputted homogeneous transform. Note that the
// inverse is not just the inverse of the matrix, but a reversed rotation, and a
// reversed linear translation. See definition for more details
Eigen::Matrix4f homogInverse(const Eigen::Matrix4f& ht);


// Returns the homogeneous transformation matrix representing the coordinate
// system and position of the right back leg of a quadruped. Assumes legs are
// positioned in the corner of a rectangular plane defined by a length and
// height. Requires the homogeneous transform representing the body center.
Eigen::Matrix4f htLegRightBack(const Eigen::Matrix4f& ht_body_center,
                               float body_length, float body_width);


// Returns the homogeneous transformation matrix representing the coordinate
// system and position of the right front leg of a quadruped. Assumes legs are
// positioned in the corner of a rectangular plane defined by a length and
// height. Requires the homogeneous transform representing the body center.
Eigen::Matrix4f htLegRightFront(const Eigen::Matrix4f& ht_body_center,
                                float body_length, float body_width);

// Returns the homogeneous transformation matrix representing the coordinate
// system and position of the left front leg of a quadruped. Assumes legs are
// positioned in the corner of a rectangular plane defined by a length and
// height. Requires the homogeneous transform representing the body center.
Eigen::Matrix4f htLegLeftFront(const Eigen::Matrix4f& ht_body_center,
                               float body_length, float body_width);

// Returns the homogeneous transformation matrix representing the coordinate
// system and position of the left back leg of a quadruped. Assumes legs are
// positioned in the corner of a rectangular plane defined by a length and
// height. Requires the homogeneous transform representing the body center.
Eigen::Matrix4f htLegLeftBack(const Eigen::Matrix4f& ht_body_center,
                              float body_length, float body_width);


// Returns the homogeneous transformation matrix for joint 0 to 1 for a
// quadruped leg
Eigen::Matrix4f ht0To1(float rot_ang, float link_length);


// Returns the homogeneous transformation matrix for joint 1 to 2 for a
// quadruped leg
Eigen::Matrix4f ht1To2();

// Returns the homogeneous transformation matrix for joint 2 to 3 for a
// quadruped leg
Eigen::Matrix4f ht2To3(float rot_ang, float link_length);

// Returns the homogeneous transformation matrix for joint 3 to 4 for a
// quadruped leg
Eigen::Matrix4f ht3To4(float rot_ang, float link_length);

// Returns the homogeneous transformation matrix from joint 0 to 1 for a
// quadruped leg
Eigen::Matrix4f ht0To4(const JointAngles& joint_angles, 
                       const LinkLengths& link_lengths);

// Returns the joint angles of a leg after running the inverse kinematics on a
// quadruped leg.
// Last argument, an optional boolean input, specifies whether the equations for
// legs 1 and 2 are used (as opposed for legs 3 and 4)
 JointAngles ikine(const Point& point, const LinkLengths& link_lengths,
                   bool is_leg_12 = true); 


}
