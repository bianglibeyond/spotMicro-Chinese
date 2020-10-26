#include "spot_micro_kinematics/utils.h"

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Geometry>

using namespace Eigen;

namespace smk
{

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang)
{
  // Create 3d transformation, and execute x, y, and z rotations
  Transform<float, 3, Affine> t = Transform<float,3,Affine>::Identity();
  t.rotate(AngleAxisf(x_ang, Vector3f::UnitX()));
  t.rotate(AngleAxisf(y_ang, Vector3f::UnitY()));
  t.rotate(AngleAxisf(z_ang, Vector3f::UnitZ()));

  return t.matrix();
}


Matrix4f homogTransXyz(float x, float y, float z)
{
  // Create a linear translation homogenous transformation matrix
  Transform<float, 3, Affine> t;
  t = Translation<float, 3> (Vector3f(x,y,z));

  return t.matrix();
}


Matrix4f homogInverse(const Matrix4f& ht)
{
//The inverse of a homogeneous transformation matrix can be represented as a
//    a matrix product of the following:
//
//                -------------------   ------------------- 
//                |           |  0  |   | 1   0   0  -x_t |
//    ht_inv   =  |   R^-1    |  0  | * | 0   1   0  -y_t |
//                |___________|  0  |   | 0   0   1  -z_t |
//                | 0   0   0 |  1  |   | 0   0   0   1   |
//                -------------------   -------------------
//
//    Where R^-1 is the inverse of the rotation matrix portion of the homogeneous
//    transform (the first three rows and columns). Note that the inverse
//    of a rotation matrix is equal to its transpose. And x_t, y_t, z_t are the
//    linear trasnformation portions of the original transform.  

  Matrix3f temp_rot = ht.block<3,3>(0,0); // Get rotation matrix portion from homogeneous transform via block
  temp_rot.transposeInPlace();    // Transpose, equivalent to inverse for rotation matrix

  // Store linear translation portion and negate directions
  Vector3f temp_translate = ht.block<3,1>(0,3);
  temp_translate = temp_translate * -1.0f;

  // Create left hand portion of ht_inv from comment block above
  Matrix4f ht_inverted1 = Matrix4f::Identity();
  ht_inverted1.block<3,3>(0,0) = temp_rot;

  // Create right hand portion of ht_in from comment block above
  Matrix4f ht_inverted2 = Matrix4f::Identity();
  ht_inverted2.block<3,1>(0,3) = temp_translate;

  // Return product of matrices, the homogeneous transform inverse
  return (ht_inverted1 * ht_inverted2);
}


Matrix4f htLegRightBack(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg rightback ht
  return (ht_body_center * htLegRightBack);
}


Matrix4f htLegRightFront(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegRightBack);
}

Matrix4f htLegLeftFront(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftFront = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftFront.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftFront);
}


Matrix4f htLegLeftBack(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftBack = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftBack);
}


Matrix4f ht0To1(float rot_ang, float link_length) {
  
  // Build up the matrix as from the paper
  Matrix4f ht_0_to_1 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_0_to_1(0,3) = -link_length*cos(rot_ang);
  ht_0_to_1(1,3) = -link_length*sin(rot_ang);

  return ht_0_to_1;
}

Matrix4f ht1To2() {
  // Build up the matrix as from the paper
  Matrix4f ht_1_to_2;

  ht_1_to_2 <<
      0.0f,   0.0f,   -1.0f,   0.0f,
     -1.0f,   0.0f,    0.0f,   0.0f,
      0.0f,   1.0f,    0.0f,   0.0f,
      0.0f,   0.0f,    0.0f,   1.0f;

  return ht_1_to_2;
}


Matrix4f ht2To3(float rot_ang, float link_length) {
  
  // Build up the matrix as from the paper
  Matrix4f ht_2_to_3 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_2_to_3(0,3) = link_length*cos(rot_ang);
  ht_2_to_3(1,3) = link_length*sin(rot_ang);

  return ht_2_to_3;
}

Matrix4f ht3To4(float rot_ang, float link_length) {
  // Same as the 2 to 3 transformation, so just call that function
  
  return ht2To3(rot_ang, link_length);
}

Matrix4f ht0To4(const JointAngles& joint_angles,
                const LinkLengths& link_lengths) {
  // Result is a sequential multiplication of all 4 transform matrices
  return (ht0To1(joint_angles.ang1, link_lengths.l1) *
          ht1To2() *
          ht2To3(joint_angles.ang2, link_lengths.l2) *
          ht3To4(joint_angles.ang3,  link_lengths.l3));
}


JointAngles ikine(const Point& point, const LinkLengths& link_lengths, bool is_leg_12) {
  using namespace std;

  // Initialize return struct
  JointAngles joint_angles;

  // Convenience variables for math
  float x4 = point.x;
  float y4 = point.y;
  float z4 = point.z;
  float l1 = link_lengths.l1;
  float l2 = link_lengths.l2;
  float l3 = link_lengths.l3;
  
  // Supporting variable D
  float D = (x4*x4 + y4*y4 + z4*z4 - l1*l1 - l2*l2 - l3*l3) /
            (2*l2*l3);
  
  // Poor man's inverse kinematics reachability protection:
  // Limit D to a maximum value of 1, otherwise the square root functions
  // below (sqrt(1 - D^2)) will attempt a square root of a negative number
  if (D > 1.0f) {
    D = 1.0f;
  } else if (D < -1.0f) {
    D = -1.0f;
  }

  if (is_leg_12) {
    joint_angles.ang3 = atan2(sqrt(1 - D*D), D);
  } else {
    joint_angles.ang3 = atan2(-sqrt(1 - D*D), D);
  }

  // Another poor mans reachability sqrt protection
  float protected_sqrt_val = x4*x4 + y4*y4 - l1*l1;
  if (protected_sqrt_val < 0.0f) { protected_sqrt_val = 0.0f;}

  joint_angles.ang2 = atan2(z4, sqrt(protected_sqrt_val)) -
         atan2(l3*sin(joint_angles.ang3), l2 + l3*cos(joint_angles.ang3));

  joint_angles.ang1 = atan2(y4, x4) + atan2(sqrt(protected_sqrt_val), -l1);

  return joint_angles;
} 










}
