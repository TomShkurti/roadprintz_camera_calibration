// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A simple example of using the Ceres minimizer.
//
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// automatic differentiation.

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
//#include <ceres/rotation.h>
//#include "ceres_costs_utils.hpp"
//#include "basic_types.h"

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "calibration_file_reader.cpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


typedef double* P_BLOCK;
//
      // iether creates a place to hold all usefull information about images or groups all of the valuable information into p_blocks
      //P_BLOCK intrinsics = camera_->camera_parameters_.pb_intrinsics;
      //P_BLOCK extrinsics = target_to_camera_poses[scene_].pb_pose;
      
/*! Brief Point3d defines a ceres_structure for a point in 3D space */
typedef struct
{
  union
  {
    struct
    {
      double x; /**< position x */
      double y; /**< position y */
      double z; /**< position z */
    };
    double pb[3]; /**< a parameter block with all three elements */
  };
} Point3d;

/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point
 *  @param angle_axis, ax, ay, and az
 *  @param tx translation tx, ty and tz
 *  @param point, the original point
 *  @param t_point, the transformed point
 */
template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3]);
template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

/*! \brief ceres compliant function to compute the residual from a distorted pinhole camera model
 *  @param point[3] the input point
 *  @param k1 radial distortion parameter k1
 *  @param k2 radial distortion parameter k2
 *  @param k3 radial distortion parameter k3
 *  @param p1 tangential distortion parameter p1
 *  @param p2 tangential distortion parameter p2
 *  @param fx focal length in x
 *  @param fy focal length in y
 *  @param cx optical center in x
 *  @param cy optical center in y
 *  @param ox observation in x
 *  @param oy observation in y
 *  @param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraPntResidualDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T& ox, T& oy,
                           T residual[2]);
template <typename T>
inline void cameraPntResidualDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T& ox,
                                  T& oy, T residual[2])
{
	/*std::cout<< "Goal u " << ox << "\n";
	std::cout<< "Goal v " << oy << "\n";*/
	
	//std::cout<< "FL in " << fx << "\n";

  //Projection
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];
  T up;
  T vp;
  
  up = cx * zp1 + fx * xp1;
  vp = cy * zp1 + fy * yp1;
  
  if(zp1 != T(0)){
  	up = up / zp1;
  	vp = vp / zp1;
  }
  
  //Distortion.
  T up_shrunk = up / cx - T(1.0);
  T vp_shrunk = vp / cy - T(1.0);
  
  T xp2 = up_shrunk * up_shrunk;  /* x^2 */
  T yp2 = vp_shrunk * vp_shrunk;  /* y^2 */
  T r2 = xp2 + yp2; /* r^2 radius squared */
  T r4 = r2 * r2;   /* r^4 */
  T r6 = r2 * r4;   /* r^6 */

  /* apply the distortion coefficients to refine pixel location */
  T xpp = up_shrunk + up_shrunk * (
  	k1 * r2           // 2nd order term
          + k2 * r4              // 4th order term
          + k3 * r6
          )              // 6th order term
          + p2 * (r2 + T(2.0) * xp2)  // tangential
          + p1 * up_shrunk * vp_shrunk * T(2.0);    // other tangential term
          ;
  T ypp = vp_shrunk + vp_shrunk * (
  	 k1 * r2           // 2nd order term
          + k2 * r4              // 4th order term
          + k3 * r6              // 6th order term
        )
          + p1 * (r2 + T(2.0) * yp2)  // tangential term
          + p2 * up_shrunk * vp_shrunk * T(2.0);    // other tangential term
          ;
          
  T up_dist = (xpp + T(1.0)) * cx;
  T vp_dist = (ypp + T(1.0)) * cy;
  
  residual[0] = up_dist - ox;
  residual[1] = vp_dist - oy;
  
  

  
  //std::cout << "3D: " << xp1 << " " << yp1 << " " << zp1 << ", ground: " << ox << " " << oy << " " << " calc: " << xpp << " " << ypp;
  //std::getchar();
}

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
// Estimate the axis of motion as part of the optimization for intrinsic cal using target on a rail
template <typename T>
void extractCameraIntrinsics(const T intrinsics[9], T& fx, T& fy, T& cx, T& cy, T& k1, T& k2, T& k3, T& p1, T& p2);
template <typename T>
inline void extractCameraIntrinsics(const T intrinsics[9], T& fx, T& fy, T& cx, T& cy, T& k1, T& k2, T& k3, T& p1,
                                    T& p2)
{
  fx = intrinsics[0]; /** focal length x */
  fy = intrinsics[1]; /** focal length y */
  cx = intrinsics[2]; /** central point x */
  cy = intrinsics[3]; /** central point y */
  k1 = intrinsics[4]; /** distortion k1  */
  k2 = intrinsics[5]; /** distortion k2  */
  k3 = intrinsics[6]; /** distortion k3  */
  p1 = intrinsics[7]; /** distortion p1  */
  p2 = intrinsics[8]; /** distortion p2  */
}

class RailICal5
{
public:
  RailICal5(double ob_x, double ob_y, double rail_position_x, double rail_position_y,double rail_position_z, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_x_(rail_position_x), rail_position_y_(rail_position_y),rail_position_z_(rail_position_z),point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, // Intrinsics (9 params)
                  const T* const c_p2, // Target origin (6 params)
                  const T* const c_p3, // Camera skew (3 params)
                  T* residual) const
  {
  //std::cout << c_p1[0] << "\n";
  	//How do I even verify this??
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    
    //cout<<"RailICal5 operator: fx, fy, cx, cy, = "<<fx<<", "<<fy<<", "<<cx<<", "<<cy<<endl;
    //Why was this originally reversed??
    //const T *target_aa(& c_p2[0]); // extract target's angle axis
    //const T *target_tx(& c_p2[3]); // extract target's position
    const T *target_aa(& c_p2[3]); // extract target's angle axis
    const T *target_tx(& c_p2[0]); // extract target's position

    // 1. Estimating the axis of motion (relative to initial target pose)
    T nominal_axis_x[3]; // Nominally we move along camera x, y and z axes
    T nominal_axis_y[3];
    T nominal_axis_z[3];

    nominal_axis_x[0] = T(1.0);
    nominal_axis_x[1] = T(0.0);
    nominal_axis_x[2] = T(0.0);
    
    nominal_axis_y[0] = T(0.0);
    nominal_axis_y[1] = T(1.0);
    nominal_axis_y[2] = T(0.0);    
    
    nominal_axis_z[0] = T(0.0);
    nominal_axis_z[1] = T(0.0);
    nominal_axis_z[2] = T(1.0);

    T rotation_axis[3]; // Here we skew the axes of motion w/ 3 rotation params
    rotation_axis[0] = c_p3[0];
    rotation_axis[1] = c_p3[1];
    rotation_axis[2] = c_p3[2]; //T(0.0);

    T motion_axis_x[3];
    T motion_axis_y[3];
    T motion_axis_z[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_x, motion_axis_x);
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_y, motion_axis_y);
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_z, motion_axis_z);

    // 2. Move the target point backwards along the rail
    //generalize...silly long-hand version of matrix multiply for R*dp
    //wsn: really I am expressing this in a frame defined based on zero mill displacement of target
    // target axes along rows/cols of circles, and target-z normal to the target plane
    // maybe call this target0 frame
    T rail_point[3]; /** point in camera coordinates */
    rail_point[0] = point_.x + T(rail_position_x_) * motion_axis_x[0]  + T(rail_position_y_) * motion_axis_y[0] + T(rail_position_z_) * motion_axis_z[0] ;
    rail_point[1] = point_.y - T(rail_position_x_) * motion_axis_x[1]  + T(rail_position_y_) * motion_axis_y[1] + T(rail_position_z_) * motion_axis_z[1] ;
    rail_point[2] = point_.z - T(rail_position_x_) * motion_axis_x[2]  + T(rail_position_y_) * motion_axis_y[2] + T(rail_position_z_) * motion_axis_z[2] ;

    /** transform point in rail coordinates into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    //wsn: NEED FOLLOWING FNC
    transformPoint(target_aa, target_tx, rail_point, camera_point);
    
    
    
    //cout<<"RailICal5: rail_point: "<<rail_point[0]<<","<<rail_point[1]<<", "<<rail_point[2]<<endl;

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    
    //TODO: Back-check if these are passing through the transform converter properly when not identity.
    //wsn: NEED FOLLOWING FNC
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);
    //cout<<"camera_point: "<<camera_point[0]<<","<<camera_point[1]<<endl;
    //cout<<"fx, fy, cx, cy, ox, oy = "<<fx<<", "<<fy<<", "<<cx<<", "<<cy<<", "<<ox<<", "<<oy<<endl;

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, double rail_position_x, double rail_position_y, double rail_position_z, Point3d point)
  {
    return new ceres::AutoDiffCostFunction<RailICal5, 2, 9, 6, 3>(new RailICal5(o_x, o_y, rail_position_x, rail_position_y, rail_position_z, point));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_x_;
  double rail_position_y_;
  double rail_position_z_; /** location of camera along rail/sled */
  Point3d point_; /** point expressed in target coordinates */
};
  

struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

/*
  double image_x = camera_observations[k].image_loc_x;
          double image_y = camera_observations[k].image_loc_y;
          Point3d point  = camera_observations[k].target->pts_[camera_observations[k].point_id];
          if(k==10) ROS_ERROR("target point %d = %8.3lf %8.3lf %8.3lf observed at %8.3lf %8.3lf",camera_observations[k].point_id, point.x, point.y, point.z, image_x, image_y);
          CostFunction *cost_function = industrial_extrinsic_cal::RailICal5::Create(image_x, image_y, Dist, point);
          P_->AddResidualBlock(cost_function, NULL, intrinsics, extrinsics, ax_ay_);
 */

int old_main(const std::string & in, const int s, const int d) {

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;
  std::vector<Eigen::Vector2d> xy_pixels_vec; 
  std::vector<Eigen::Vector2d> xy_targets_vec; 
  std::vector<Eigen::Vector3d> xyz_sled_vec;
  Eigen::Vector2d xy_pixels,xy_targets;
  Eigen::Vector3d xyz_sled;
  //read the calibration file:
  if (!read_calibration_file(in, xy_pixels_vec,xy_targets_vec, xyz_sled_vec )) {
      cout<<"could not open file "<<in<<"; quitting"<<endl;
      return 1;
  }
  cout<<"calibration file has been read in"<<endl;
  // Build the problem.
  Problem problem;
  //bool read_paintfile(std::string fname, std::vector<Eigen::Vector2d> &xy_pixels_vec, std::vector<Eigen::Vector2d> &xy_targets_vec, std::vector<Eigen::Vector3d> &xyz_sled_vec ) {

  
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  
  /*
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);
  */
  
  /*
   for (int k = 0; k < num_observations; k++)
        {
          double image_x = camera_observations[k].image_loc_x;
          double image_y = camera_observations[k].image_loc_y;
          Point3d point  = camera_observations[k].target->pts_[camera_observations[k].point_id];
          if(k==10) ROS_ERROR("target point %d = %8.3lf %8.3lf %8.3lf observed at %8.3lf %8.3lf",camera_observations[k].point_id, point.x, point.y, point.z, image_x, image_y);
          CostFunction *cost_function = industrial_extrinsic_cal::RailICal5::Create(image_x, image_y, Dist, point);
          P_->AddResidualBlock(cost_function, NULL, intrinsics, extrinsics, ax_ay_);
        }  // for each observation at this camera_location
   */
  double image_x,image_y,target_x,target_y,sled_x,sled_y,sled_z;  //these values are all known, corresponding to a calibration data point
  Point3d point;
  int nlines = (int) xy_pixels_vec.size();
  
  double intrinsics[9],G_cm[6],G_st[3];  //these values are all parameters to be discovered:
    //intrinsics: 
    //	     focal_length_x,
    //       focal_length_y,
    //	      center_x,
    //	      center_y,
    //	      distortion_k1,
    //	      distortion_k2,
    //	      distortion_k3,
    //	      distortion_p1,
    //	      distortion_p2;
    
    intrinsics[0]=500; //guess at the focal length, in pixels
    intrinsics[1]=500;
    intrinsics[2]= 640/2 ; //704x480
    intrinsics[3] = 640/2;
    for (int i=4;i<9;i++) intrinsics[i]=0.0; //init distortion to zero
    
    for (int i=3;i<6;i++) G_cm[i]=0.0; //init approx rotation from mill frame to target frame = 0 angle/axis
    //Init approx translation starts @ identity.
    G_cm[0] = 0.0;
    G_cm[1] = 0.0;
    G_cm[2] = 0.0;
    
    for (int i=0;i<3;i++) G_st[i]=0.0; //assume and sled frames are approximately aligned;
  
  for (int i=0;i<nlines;i++) {
    xy_pixels = xy_pixels_vec[i];
    xy_targets = xy_targets_vec[i];
    xyz_sled = xyz_sled_vec[i];
    point.x= xy_targets[0];
    point.y= xy_targets[1];
    point.z = 0.0;
  
    CostFunction *cost_function = RailICal5::Create(xy_pixels[0],xy_pixels[1],xyz_sled[0],xyz_sled[1],xyz_sled[2],point);
    //problem.AddResidualBlock(cost_function, NULL, &x);
    
    problem.AddResidualBlock(cost_function, NULL, intrinsics, G_cm, G_st);
  }
  
  // Run the solver!
  Solver::Options options;
  
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n\n\n";
  
  std::printf("F_x %f F_y %f c_x %f c_y %f\n\n", intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
  std::printf("k1 %f k2 %f k3 %f p1 %f p2 %f \n\n", intrinsics[4], intrinsics[5], intrinsics[6], intrinsics[7], intrinsics[8]);
  std::printf("x_c %f y_c %f z_c %f r_c %f p_c %f w_c %f\n\n", G_cm[0], G_cm[1], G_cm[2], G_cm[3], G_cm[4], G_cm[5]);
  std::printf("r_t %f p_t %f w_t %f\n\n", G_st[0], G_st[1], G_st[2]);

  std::ofstream output;
  output.open("/home/tes77/columnarity_undistorted.csv", std::ofstream::out | std::ofstream::app);
  output
  	<< std::to_string(intrinsics[0]) << ", "
  	<< std::to_string(intrinsics[4]) << ", "
  	<< std::to_string(summary.final_cost) << ", "
  	<< std::to_string(nlines) << ", "
  	<< std::to_string(s) << ", "
  	<< std::to_string(d) << "\n";
  	
  output.close();
  
  return 0;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  
  for(int s = 1; s <= 3; s ++){
  	for(int d = 0; d <= 95; d += 5){
  		std::string the_filename =
  			"/home/tes77/columnarity/undistorted_" + std::to_string(s)
  			+ "_" + std::to_string(d) + ".csv";
  		
  		old_main(the_filename, s, d);
  	}
  }
  
  return 0;
}
