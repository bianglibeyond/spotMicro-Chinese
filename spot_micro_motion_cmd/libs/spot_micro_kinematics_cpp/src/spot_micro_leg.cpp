#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"

using namespace Eigen;

namespace smk {


// Constructor
SpotMicroLeg::SpotMicroLeg(const JointAngles& joint_angles,
                           const LinkLengths& link_lengths,
                           bool is_leg_12)
    : joint_angles_(joint_angles),
      link_lengths_(link_lengths),
      is_leg_12_(is_leg_12) {
}


void SpotMicroLeg::setAngles(const JointAngles& joint_angles) {
  // Update object's joint angles
  
  joint_angles_ = joint_angles;
}


void SpotMicroLeg::setFootPosLocalCoordinates(const Point& point) {

  // Run inverse kinematics to find joint angles
  JointAngles joint_angles = ikine(point, link_lengths_, is_leg_12_);

  // Call method to set joint angles of the leg
  setAngles(joint_angles);
}


void SpotMicroLeg::setFootPosGlobalCoordinates(const Point& point, 
                                               const Matrix4f& ht_leg_start) {

  // Need to express the point in the leg's coordinate system, can do so by
  // transforming a vector of the points in global coordinate by the inverse of
  // the leg's starting homogeneous transform

  // Make a homogeneous vector, and store the point in global coords in it
  Eigen::Vector4f p4_ht_vec(point.x, point.y, point.z, 1.0f);

  // Multiply it by the inverse of the homgeneous transform of the leg start.
  // This operation yields a foot position in the foot's local coordinates
  p4_ht_vec = homogInverse(ht_leg_start) * p4_ht_vec; 

  Point point_local{.x = p4_ht_vec(0), .y = p4_ht_vec(1), .z = p4_ht_vec(2)};

  // Call this leg's method for setting foot position in local cordinates
  setFootPosLocalCoordinates(point_local);
}


Point SpotMicroLeg::getFootPosGlobalCoordinates(const Matrix4f& ht_leg_start) {
 // Get homogeneous transform of foot
  Matrix4f ht_foot = ht_leg_start * 
                     ht0To4(joint_angles_, link_lengths_); 

  // Construct return point structure
  Point return_point = {.x = ht_foot(0,3),
                        .y = ht_foot(1,3),
                        .z = ht_foot(2,3) };
  return return_point;
}


JointAngles SpotMicroLeg::getLegJointAngles() {
  return joint_angles_;
}


}
