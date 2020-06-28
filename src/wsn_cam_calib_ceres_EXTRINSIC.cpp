// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
// used to endorse or promote products derived from this software without
// specific prior written permission.
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

#include "calibration_file_reader_EXTRINSIC.cpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// This is how Newman converts to AA. Potentially suspect.
/*T nominal_axis_x[3]; // Nominally we move along camera x, y and z axes
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
ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis_z, motion_axis_z);*/

inline double dtor(double d){
	return (d * 3.14159) / 180.0;
}

template <typename T>
inline void transformPoint_rm(const T translation[3], const T rotation[9], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_rm(const T translation[3], const T rotation[9], const T point_in[3], T point_out[3]){
	point_out[0] = point_in[0] * rotation[0] + point_in[1] * rotation[1] + point_in[2] * rotation[2] + translation[0];
	point_out[1] = point_in[0] * rotation[3] + point_in[1] * rotation[4] + point_in[2] * rotation[5] + translation[1];
	point_out[2] = point_in[0] * rotation[6] + point_in[1] * rotation[7] + point_in[2] * rotation[8] + translation[2];
}

/*! \brief ceres compliant function to apply a rotation and translation to transform a point
 *@param rotation r, p, w
 *@param translation x, y and z
 *@param point_in, the original point
 *@param point_out, the transformed point
 */
template <typename T>
inline void transformPoint_euler(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_euler(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]){
	T R [9];
	ceres::EulerAnglesToRotationMatrix(rotation, 3, R);
	
	transformPoint_rm(translation, R, point_in, point_out);
}

/*! \brief ceres compliant function to apply a rotation and translation to transform a point
 *@param rotation angle axis x, y, z, norm is amount
 *@param translation x, y and z
 *@param point_in, the original point
 *@param point_out, the transformed point
 */
template <typename T>
inline void transformPoint_aa(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]);
template <typename T>
inline void transformPoint_aa(const T translation[3], const T rotation[3], const T point_in[3], T point_out[3]){
	std::cout << "\t" << translation[0] << "\n";
	std::cout << "\t" << translation[1] << "\n";
	std::cout << "\t" << translation[2] << "\n\n";
	std::cout << "\t" << rotation[0] << "\n";
	std::cout << "\t" << rotation[1] << "\n";
	std::cout << "\t" << rotation[2] << "\n\n";

	std::cout << "\t" << point_in[0] << "\n";
	std::cout << "\t" << point_in[1] << "\n";
	std::cout << "\t" << point_in[2] << "\n\n";
	

	ceres::AngleAxisRotatePoint(rotation, point_in, point_out);
	point_out[0] = point_out[0] + translation[0];
	point_out[1] = point_out[1] + translation[1];
	point_out[2] = point_out[2] + translation[2];
	
	
	std::cout << "\t" << point_out[0] << "\n";
	std::cout << "\t" << point_out[1] << "\n";
	std::cout << "\t" << point_out[2] << "\n\n";
}

/*! \brief ceres compliant function to compute the residual from a distorted pinhole camera model
 *@param point_x the input point x
 *@param point_y the input point y
 *@param point_z the input point z
 *@param k1 radial distortion parameter k1
 *@param k2 radial distortion parameter k2
 *@param k3 radial distortion parameter k3
 *@param p1 tangential distortion parameter p1
 *@param p2 tangential distortion parameter p2
 *@param fx focal length in x
 *@param fy focal length in y
 *@param cx optical center in x
 *@param cy optical center in y
 *@param ox observation in x
 *@param oy observation in y
 *@param residual the output or difference between where the point should appear given the parameters, and where it
 * was observed
 */
template <typename T>
void cameraPntResidualDist(
	T& point_x, T& point_y, T& point_z,
	T k1, T k2, T k3, T p1, T p2,
	T fx, T fy, T cx, T cy,
	T ox, T oy,
	T residual[2]
);
template <typename T>
inline void cameraPntResidualDist(
	T& point_x, T& point_y, T& point_z,
	T k1, T k2, T k3, T p1, T p2,
	T fx, T fy, T cx, T cy,
	T ox, T oy,
	T residual[2]
){
	//std::cout<< "Goal u " << ox << "\n";
	//std::cout<< "Goal v " << oy << "\n";
	
	//std::cout<< "FL in " << fx << "\n";

	//Projection
	T xp1 = point_x;
	T yp1 = point_y;
	T zp1 = point_z;
	T up;
	T vp;
	
	//std::cout << point_x << "\n";
	//std::cout << point_y << "\n";
	//std::cout << point_z << "\n\n";
	
	up = cx * zp1 + fx * xp1;
	vp = cy * zp1 + fy * yp1;

	if(zp1 != T(0)){
		up = up / zp1;
		vp = vp / zp1;
	}

	//Distortion.
	T up_shrunk = up / cx - T(1.0);
	T vp_shrunk = vp / cy - T(1.0);

	T xp2 = up_shrunk * up_shrunk;// x^2 
	T yp2 = vp_shrunk * vp_shrunk;// y^2 
	T r2 = xp2 + yp2; // r^2 radius squared
	T r4 = r2 * r2; // r^4 
	T r6 = r2 * r4; // r^6 

	// apply the distortion coefficients to refine pixel location 
	T xpp = up_shrunk + up_shrunk * (
			k1 * r2 // 2nd order term
			+ k2 * r4// 4th order term
			+ k3 * r6
		)// 6th order term
		+ p2 * (r2 + T(2.0) * xp2)// tangential
		+ p1 * up_shrunk * vp_shrunk * T(2.0);// other tangential term
	;
	T ypp = vp_shrunk + vp_shrunk * (
			k1 * r2 // 2nd order term
			+ k2 * r4// 4th order term
			+ k3 * r6// 6th order term
		)
		+ p1 * (r2 + T(2.0) * yp2)// tangential term
		+ p2 * up_shrunk * vp_shrunk * T(2.0);// other tangential term
	;

	T up_dist = (xpp + T(1.0)) * cx;
	T vp_dist = (ypp + T(1.0)) * cy;

	residual[0] = up_dist - ox;
	residual[1] = vp_dist - oy;
	
	//residual[0] = up - ox;
	//residual[1] = vp - oy;
	
	//std::cout << up << "\n";
	//std::cout << vp << "\n";

	//std::getchar();
}

class ExtrinsicCalEntry{
public:
	// A templated cost functor that implements the residual
	template<typename T>
	bool operator()(//TODO Why are all these const / should all these be const?
		const T* FLANGE_to_HEAD_t, const T* FLANGE_to_HEAD_r,
		const T* CAM_to_FOREARM_t, const T* CAM_to_FOREARM_r,
	T* residual) const {

	
	//1: Transform target points into camera frame
	//	CAM_to_POINT = AM_to_FOREARM * FOREARM_to_BASE * BASE_to_FLANGE * FLANGE_to_HEAD * HEAD_to_POINT
	
	//	1a: BASE_to_POINT
	T HEAD_to_POINT [3];
	HEAD_to_POINT[0] = T(0.0);
	HEAD_to_POINT[1] = T(0.0);
	HEAD_to_POINT[2] = T(0.2);
	
	/*std::cout << "HEAD TO POINT\n";
	std::cout << HEAD_to_POINT[0] << "\n";
	std::cout << HEAD_to_POINT[1] << "\n";
	std::cout << HEAD_to_POINT[2] << "\n\n";*/
	
	//	1b: FLANGE_to_HEAD * HEAD_to_POINT
	T FLANGE_to_POINT [3];
	transformPoint_euler(FLANGE_to_HEAD_t, FLANGE_to_HEAD_r, HEAD_to_POINT, FLANGE_to_POINT);
	
	/*std::cout << "FLANGE TO POINT\n";
	std::cout << FLANGE_to_POINT[0] << "\n";
	std::cout << FLANGE_to_POINT[1] << "\n";
	std::cout << FLANGE_to_POINT[2] << "\n\n";*/
	
	//	1d: BASE_to_FLANGE * FLANGE_to_HEAD * HEAD_to_POINT
	T BASE_to_FLANGE_normed [9];
	BASE_to_FLANGE_normed[0] = T(BASE_to_FLANGE_r[0]);
	BASE_to_FLANGE_normed[1] = T(BASE_to_FLANGE_r[1]);
	BASE_to_FLANGE_normed[2] = T(BASE_to_FLANGE_r[2]);
	BASE_to_FLANGE_normed[3] = T(BASE_to_FLANGE_r[3]);
	BASE_to_FLANGE_normed[4] = T(BASE_to_FLANGE_r[4]);
	BASE_to_FLANGE_normed[5] = T(BASE_to_FLANGE_r[5]);
	BASE_to_FLANGE_normed[6] = T(BASE_to_FLANGE_r[6]);
	BASE_to_FLANGE_normed[7] = T(BASE_to_FLANGE_r[7]);
	BASE_to_FLANGE_normed[8] = T(BASE_to_FLANGE_r[8]);
	
	T BASE_to_FLANGE_translation [3];
	BASE_to_FLANGE_translation[0] = T(BASE_to_FLANGE_x);
	BASE_to_FLANGE_translation[1] = T(BASE_to_FLANGE_y);
	BASE_to_FLANGE_translation[2] = T(BASE_to_FLANGE_z);

	T BASE_to_POINT [3];
	transformPoint_rm(BASE_to_FLANGE_translation, BASE_to_FLANGE_normed, FLANGE_to_POINT, BASE_to_POINT);
	
	/*std::cout << "BASE TO POINT\n";
	std::cout << BASE_to_POINT[0] << "\n";
	std::cout << BASE_to_POINT[1] << "\n";
	std::cout << BASE_to_POINT[2] << "\n\n";*/
	
	T FOREARM_to_BASE_normed [9];
	FOREARM_to_BASE_normed[0] = T(FOREARM_to_BASE_r[0]);
	FOREARM_to_BASE_normed[1] = T(FOREARM_to_BASE_r[1]);
	FOREARM_to_BASE_normed[2] = T(FOREARM_to_BASE_r[2]);
	FOREARM_to_BASE_normed[3] = T(FOREARM_to_BASE_r[3]);
	FOREARM_to_BASE_normed[4] = T(FOREARM_to_BASE_r[4]);
	FOREARM_to_BASE_normed[5] = T(FOREARM_to_BASE_r[5]);
	FOREARM_to_BASE_normed[6] = T(FOREARM_to_BASE_r[6]);
	FOREARM_to_BASE_normed[7] = T(FOREARM_to_BASE_r[7]);
	FOREARM_to_BASE_normed[8] = T(FOREARM_to_BASE_r[8]);
	
	T FOREARM_to_BASE_translation [3];
	FOREARM_to_BASE_translation[0] = T(FOREARM_to_BASE_x);
	FOREARM_to_BASE_translation[1] = T(FOREARM_to_BASE_y);
	FOREARM_to_BASE_translation[2] = T(FOREARM_to_BASE_z);

	T FOREARM_to_POINT [3];
	transformPoint_rm(FOREARM_to_BASE_translation, FOREARM_to_BASE_normed, BASE_to_POINT, FOREARM_to_POINT);
	
	/*std::cout << FOREARM_to_POINT[0] << "\n";
	std::cout << FOREARM_to_POINT[1] << "\n";
	std::cout << FOREARM_to_POINT[2] << "\n\n";*/

	//	1e: CAM_to_FOREARM * FOREARM_to_BASE * BASE_to_POINT
	T CAM_to_POINT [3];
	transformPoint_euler(CAM_to_FOREARM_t, CAM_to_FOREARM_r, FOREARM_to_POINT, CAM_to_POINT);
	
	/*std::cout << "CAM TO POINT\n";
	std::cout << CAM_to_POINT[0] << "\n";
	std::cout << CAM_to_POINT[1] << "\n";
	std::cout << CAM_to_POINT[2] << "\n\n";*/

	// compute project point into image plane and compute residual
	T ox = T(image_pixel_u);
	T oy = T(image_pixel_v);

	//TODO: Back-check if these are passing through the transform converter properly when not identity.
	cameraPntResidualDist(
		CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
		T(k1), T(k2), T(k3), T(p1), T(p2),
		T(f_X), T(f_Y), T(c_X), T(c_Y),
		ox, oy,
		residual
	);
	
	//std::getchar();
	return true;
}
	//Member vars- all the perpoint constants
	double image_pixel_u; /** observed x location of object in image */
	double image_pixel_v; /** observed y location of object in image */
	
	double BASE_to_FLANGE_x, BASE_to_FLANGE_y, BASE_to_FLANGE_z;
	double BASE_to_FLANGE_r[9];
	double FOREARM_to_BASE_x, FOREARM_to_BASE_y, FOREARM_to_BASE_z;
	double FOREARM_to_BASE_r[9];
	
	double f_X, f_Y; /** Projection intrinsics */
	double c_X, c_Y;
	double k1, k2, k3, p1, p2; /** Distortion intrinsics */

	//Dumbest thing...
	ExtrinsicCalEntry(
		const double image_pixel_u_, const double image_pixel_v_,

		const double BASE_to_FLANGE_x_, const double BASE_to_FLANGE_y_, const double BASE_to_FLANGE_z_,
		const double BASE_to_FLANGE_r_[9],
		
		const double FOREARM_to_BASE_x_, const double FOREARM_to_BASE_y_, const double FOREARM_to_BASE_z_,
		const double FOREARM_to_BASE_r_[9],
		
		const double f_X_, const double f_Y_, const double c_X_, const double c_Y_,
		const double k1_, const double k2_, const double k3_, const double p1_, const double p2_
	):
		image_pixel_u(image_pixel_u_), image_pixel_v(image_pixel_v_),
		
		BASE_to_FLANGE_x(BASE_to_FLANGE_x_), BASE_to_FLANGE_y(BASE_to_FLANGE_y_), BASE_to_FLANGE_z(BASE_to_FLANGE_z_),
		FOREARM_to_BASE_x(FOREARM_to_BASE_x_), FOREARM_to_BASE_y(FOREARM_to_BASE_y_), FOREARM_to_BASE_z(FOREARM_to_BASE_z_),
		
		f_X(f_X_), f_Y(f_Y_), c_X(c_X_), c_Y(c_Y_),
		k1(k1_), k2(k2_), k3(k3_), p1(p1_), p2(p2_)
	{
		for(int i = 0; i < 9; i++){
			BASE_to_FLANGE_r[i] = BASE_to_FLANGE_r_[i];
			FOREARM_to_BASE_r[i] = FOREARM_to_BASE_r_[i];
		}
	}

	static ceres::CostFunction* Create(	
		const double image_pixel_u_, const double image_pixel_v_,
			
		const double BASE_to_FLANGE_x_, const double BASE_to_FLANGE_y_, const double BASE_to_FLANGE_z_,
		const double BASE_to_FLANGE_r_[9],
		const double FOREARM_to_BASE_x_, const double FOREARM_to_BASE_y_, const double FOREARM_to_BASE_z_,
		const double FOREARM_to_BASE_r_[9],
		
		//Global projection constants.
		const double f_X_, const double f_Y_, const double c_X_, const double c_Y_,
		const double k1_, const double k2_, const double k3_, const double p1_, const double p2_
	){
		return new ceres::AutoDiffCostFunction<ExtrinsicCalEntry, 2, 3, 3, 3, 3>(new ExtrinsicCalEntry(
			image_pixel_u_, image_pixel_v_,
			
			BASE_to_FLANGE_x_, BASE_to_FLANGE_y_, BASE_to_FLANGE_z_,
			BASE_to_FLANGE_r_,
			FOREARM_to_BASE_x_, FOREARM_to_BASE_y_, FOREARM_to_BASE_z_,
			FOREARM_to_BASE_r_,
			
			f_X_, f_Y_, c_X_, c_Y_,
			k1_, k2_, k3_, p1_, p2_
		));
	}
};

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	// Storage for input values constant for each point.
	std::vector<Eigen::Affine3d> BASE_to_FLANGE_vec;
	std::vector<Eigen::Affine3d> FOREARM_to_BASE_vec;
	std::vector<Eigen::Vector2d> image_pixel_vec;
	
	//read the calibration file:
	//TODO Parametrize dis
	std::string fname("/home/print/synthetic_data.csv");
	if (!read_calibration_file(fname, BASE_to_FLANGE_vec, FOREARM_to_BASE_vec, image_pixel_vec)) {
		cout<<"could not open file "<<fname<<"; quitting"<<endl;
		return 1;
	}
	cout<<"calibration file has been read in"<<endl;
	
	std::cout << "FOREARM_to_BASE\n" << FOREARM_to_BASE_vec[0].matrix() << "\n\n";
	std::cout << "BASE_to_FLANGE\n" << BASE_to_FLANGE_vec[0].matrix() << "\n\n";
	
	// Build the problem.
	Problem problem;
	int nlines = (int) image_pixel_vec.size();
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).

	//Known values (one per data point)
	double image_u,image_v;
	double FOREARM_to_BASE [16];
	double BASE_to_FLANGE [16];

	//Known values (constant)
	//TODO Initialize from an external source where the intrinsic calibration results are output.
	double projection_matrix[12] = {
		434.8289178042981,	0.0,			500.5,	0.0,
		0.0,			434.8289178042981,	330.5,	0.0,
		0.0,			0.0,			1.0,	0.0
	};
	double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

	//Unknown values
	double FLANGE_to_HEAD_t [3];
	double FLANGE_to_HEAD_r [3];
	double CAM_to_FOREARM_t [3];
	double CAM_to_FOREARM_r [3];
	
	//Initialize the unknowns
	//TODO Find a way to inform this parametrically
	//TODO For simulation tests, get these ground truths from the simulation
	FLANGE_to_HEAD_t[0] = 0.0;
	FLANGE_to_HEAD_t[1] = 0.0;
	FLANGE_to_HEAD_t[2] = 0.0;
	FLANGE_to_HEAD_r[0] = 0.0;
	FLANGE_to_HEAD_r[1] = 0.0;
	FLANGE_to_HEAD_r[2] = 0.0;
	CAM_to_FOREARM_t[0] = 0.0;
	CAM_to_FOREARM_t[1] = 0.0;
	CAM_to_FOREARM_t[2] = 0.0;
	CAM_to_FOREARM_r[0] = 0.0;
	CAM_to_FOREARM_r[1] = 0.0;
	CAM_to_FOREARM_r[2] = 0.0;
	
	for (int i=0; i<nlines; i++) {//For each data entry...
		//Add the per-point constants
		Eigen::Vector3d translation_h = BASE_to_FLANGE_vec[i].translation();
		Eigen::Matrix3d matrix_h = BASE_to_FLANGE_vec[i].linear();
		double rotation_h[9] = {
			matrix_h(0,0), matrix_h(0,1), matrix_h(0,2),
			matrix_h(1,0), matrix_h(1,1), matrix_h(1,2),
			matrix_h(2,0), matrix_h(2,1), matrix_h(2,2)
		};
		
		Eigen::Vector3d translation_c = FOREARM_to_BASE_vec[i].translation();
		Eigen::Matrix3d matrix_c = FOREARM_to_BASE_vec[i].linear();
		double rotation_c[9] = {
			matrix_c(0,0), matrix_c(0,1), matrix_c(0,2),
			matrix_c(1,0), matrix_c(1,1), matrix_c(1,2),
			matrix_c(2,0), matrix_c(2,1), matrix_c(2,2)
		};
		
		CostFunction *cost_function = ExtrinsicCalEntry::Create(
			image_pixel_vec[i][0], image_pixel_vec[i][1],
			
			translation_h[0], translation_h[1], translation_h[2],
			rotation_h,
			
			translation_c[0], translation_c[1], translation_c[2],
			rotation_c,
			
			//Global projection constants.
			projection_matrix[0], projection_matrix[5], projection_matrix[2], projection_matrix[6],
			distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]
		);
		
		//Add the parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			FLANGE_to_HEAD_t, FLANGE_to_HEAD_r,
			CAM_to_FOREARM_t, CAM_to_FOREARM_r
		);
	}
	
	
	//Bound the rotations.
	problem.SetParameterLowerBound(FLANGE_to_HEAD_r, 0, -180.0);
	problem.SetParameterUpperBound(FLANGE_to_HEAD_r, 0,  180.0);
	problem.SetParameterLowerBound(FLANGE_to_HEAD_r, 1, -180.0);
	problem.SetParameterUpperBound(FLANGE_to_HEAD_r, 1,  180.0);
	problem.SetParameterLowerBound(FLANGE_to_HEAD_r, 2, -180.0);
	problem.SetParameterUpperBound(FLANGE_to_HEAD_r, 2,  180.0);
	
	
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 0, -180.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 0,  180.0);
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 1, -180.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 1,  180.0);
	problem.SetParameterLowerBound(CAM_to_FOREARM_r, 2, -180.0);
	problem.SetParameterUpperBound(CAM_to_FOREARM_r, 2,  180.0);
	
	/*std::cout << "" << FOREARM_to_BASE_vec[0].matrix() << "\n\n";
	std::cout << "" << FOREARM_to_BASE_vec[0].linear() << "\n\n";
	
	Eigen::Matrix3d matrix_c = FOREARM_to_BASE_vec[0].linear();
		double rotation_v[9] = {
			matrix_c(0,0), matrix_c(0,1), matrix_c(0,2),
			matrix_c(1,0), matrix_c(1,1), matrix_c(1,2),
			matrix_c(2,0), matrix_c(2,1), matrix_c(2,2)
		};
	printf("%f %f %f\n", rotation_v[0], rotation_v[1], rotation_v[2]);
	printf("%f %f %f\n", rotation_v[3], rotation_v[4], rotation_v[5]);
	printf("%f %f %f\n", rotation_v[6], rotation_v[7], rotation_v[8]);*/

	// Run the solver!
	Solver::Options options;

	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 1000;

	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n\n\n";

	std::printf("FLANGE_to_HEAD:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", FLANGE_to_HEAD_t[0], FLANGE_to_HEAD_t[1], FLANGE_to_HEAD_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(FLANGE_to_HEAD_r[0]), dtor(FLANGE_to_HEAD_r[1]), dtor(FLANGE_to_HEAD_r[2]));
	
	std::printf("CAM_to_FOREARM:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", CAM_to_FOREARM_t[0], CAM_to_FOREARM_t[1], CAM_to_FOREARM_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(CAM_to_FOREARM_r[0]), dtor(CAM_to_FOREARM_r[1]), dtor(CAM_to_FOREARM_r[2]));

	return 0;
}
