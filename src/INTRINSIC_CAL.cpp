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

#include <string.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
//#include <ceres/rotation.h>
//#include "ceres_costs_utils.hpp"
//#include "basic_types.h"

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/package.h>

#include "INTRINSIC_READER.cpp"

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
	/*std::cout << "\tTRANSLATION\n";
	std::cout << "\t" << translation[0] << "\n";
	std::cout << "\t" << translation[1] << "\n";
	std::cout << "\t" << translation[2] << "\n";
	
	std::cout << "\tROTATION\n";
	std::cout << "\t" << rotation[0] << "  " << rotation[1] << "  " << rotation[2] << "\n";
	std::cout << "\t" << rotation[3] << "  " << rotation[4] << "  " << rotation[5] << "\n";
	std::cout << "\t" << rotation[6] << "  " << rotation[7] << "  " << rotation[8] << "\n";*/

	point_out[0] = (point_in[0] * rotation[0] + point_in[1] * rotation[1] + point_in[2] * rotation[2]) + translation[0];
	point_out[1] = (point_in[0] * rotation[3] + point_in[1] * rotation[4] + point_in[2] * rotation[5]) + translation[1];
	point_out[2] = (point_in[0] * rotation[6] + point_in[1] * rotation[7] + point_in[2] * rotation[8]) + translation[2];
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
	const T& point_x, const T& point_y, const T& point_z,
	const T& fx, const T& fy, const T& cx, const T& cy,
	const T& k1, const T& k2, const T& k3, const T& p1, const T& p2,
	const T& ox, const T& oy,
	T* residual
);
template <typename T>
inline void cameraPntResidualDist(
	const T& point_x, const T& point_y, const T& point_z,
	const T& fx, const T& fy, const T& cx, const T& cy,
	const T& k1, const T& k2, const T& k3, const T& p1, const T& p2,
	const T& ox, const T& oy,
	T* residual
){
	//std::cout<< "Goal u " << ox << "\n";
	//std::cout<< "Goal v " << oy << "\n";
	
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
	
	//residual[0] = up - ox;
	//residual[1] = vp - oy;

	//Distortion.
	
	T U_small = (up - cx) / (cx * T(2.0));
	T V_small = (vp - cy) / (cy * T(2.0));
	
	T r2 = pow(U_small, T(2.0)) + pow(V_small, T(2.0));
	T r4 = r2 * r2;
	T r6 = r2 * r2 * r2;
	
	T U_radial = U_small * (T(1.0) + k1 * r2 + k2 * r4 + k3 * r6);
	T V_radial = V_small * (T(1.0) + k1 * r2 + k2 * r4 + k3 * r6);
	
	T U_tangential = T(2.0) * p1 * U_small * V_small + p2 * (r2 + T(2.0) * U_small * U_small);
	T V_tangential = p1 * (r2 + T(2.0) * V_small * V_small) + T(2.0) * p2 * U_small * V_small;
	
	T U_distort = ((U_radial + U_tangential) * cx * T(2.0)) + cx;
	T V_distort = ((V_radial + V_tangential) * cy * T(2.0)) + cy;

	residual[0] = U_distort - ox;
	residual[1] = V_distort - oy;
	
	//std::cout << up << "\n";
	//std::cout << vp << "\n";

	//std::getchar();
}

class ExtrinsicCalEntry{
public:
	// A templated cost functor that implements the residual
	//Is given the UNKNOWN TERMS
	template<typename T>
	bool operator()(//TODO Why are all these const / should all these be const?
		const T* TIP_to_TARGET_translation, const T* TIP_to_TARGET_rotation,
		const T* CAM_to_BASE_translation, const T* CAM_to_BASE_rotation,
		const T* projection, const T* distortion,
	T* residual) const {
		T fx = projection[0];
		T cx = projection[1];
		T fy = projection[2];
		T cy = projection[3];

		//1: Transform target points into camera frame
		//	CAM_to_POINT = CAM_to_BASE * BASE_to_TIP * TIP_to_TARGET * TARGET_to_POINT
	
		//	1a: TARGET_to_POINT
		T TARGET_to_POINT [3];
		TARGET_to_POINT[0] = T(TARGET_to_POINT_translation[0]);
		TARGET_to_POINT[1] = T(TARGET_to_POINT_translation[1]);
		TARGET_to_POINT[2] = T(TARGET_to_POINT_translation[2]);
	
		/*std::cout << "TARGET TO POINT\n";
		std::cout << TARGET_to_POINT[0] << "\n";
		std::cout << TARGET_to_POINT[1] << "\n";
		std::cout << TARGET_to_POINT[2] << "\n\n";*/
	
		//	1b: TIP_to_TARGET * TARGET_to_POINT
		
		T TIP_to_POINT [3];
		transformPoint_euler(TIP_to_TARGET_translation, TIP_to_TARGET_rotation, TARGET_to_POINT, TIP_to_POINT);
		
		/*std::cout << "TIP TO POINT\n";
		std::cout << TIP_to_POINT[0] << "\n";
		std::cout << TIP_to_POINT[1] << "\n";
		std::cout << TIP_to_POINT[2] << "\n\n";*/
		
		//	1c: BASE_to_TIP * TIP_to_TARGET * TARGET_to_POINT
		T BASE_to_TIP_r [9];
		BASE_to_TIP_r[0] = T(BASE_to_TIP_rotation[0]);
		BASE_to_TIP_r[1] = T(BASE_to_TIP_rotation[1]);
		BASE_to_TIP_r[2] = T(BASE_to_TIP_rotation[2]);
		BASE_to_TIP_r[3] = T(BASE_to_TIP_rotation[3]);
		BASE_to_TIP_r[4] = T(BASE_to_TIP_rotation[4]);
		BASE_to_TIP_r[5] = T(BASE_to_TIP_rotation[5]);
		BASE_to_TIP_r[6] = T(BASE_to_TIP_rotation[6]);
		BASE_to_TIP_r[7] = T(BASE_to_TIP_rotation[7]);
		BASE_to_TIP_r[8] = T(BASE_to_TIP_rotation[8]);
		
		T BASE_to_TIP_t [3];
		BASE_to_TIP_t[0] = T(BASE_to_TIP_translation[0]);
		BASE_to_TIP_t[1] = T(BASE_to_TIP_translation[1]);
		BASE_to_TIP_t[2] = T(BASE_to_TIP_translation[2]);
	
		T BASE_to_POINT [3];
		transformPoint_rm(BASE_to_TIP_t, BASE_to_TIP_r, TIP_to_POINT, BASE_to_POINT);
		
		/*std::cout << "BASE TO POINT\n";
		std::cout << BASE_to_POINT[0] << "\n";
		std::cout << BASE_to_POINT[1] << "\n";
		std::cout << BASE_to_POINT[2] << "\n\n";*/
		
		//	1d: CAM_to_BASE * BASE_to_TIP * TIP_to_TARGET * TARGET_to_POINT
		T CAM_to_POINT [3];
		transformPoint_euler(CAM_to_BASE_translation, CAM_to_BASE_rotation, BASE_to_POINT, CAM_to_POINT);
		
		/*std::cout << "CAM TO POINT\n";
		std::cout << CAM_to_POINT[0] << "\n";
		std::cout << CAM_to_POINT[1] << "\n";
		std::cout << CAM_to_POINT[2] << "\n\n";*/
	
		// compute project point into image plane and compute residual
		T ox = T(image_pixels[0]);
		T oy = T(image_pixels[1]);
	
		//TODO: Back-check if these are passing through the transform converter properly when not identity.
		cameraPntResidualDist(
			CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
			fx, cx, fy, cy,
			distortion[0], distortion[1], distortion[2], distortion[3], distortion[4],
			ox, oy,
			residual
		);
		//std::getchar();
		return true;
	}
	
	//Member vars- all the perpoint constants
	double image_pixels[2]; /** observed px location of object in image */
	double BASE_to_TIP_rotation[9];
	double BASE_to_TIP_translation[3];
	double TARGET_to_POINT_rotation[9];
	double TARGET_to_POINT_translation[3];

	//Dumbest thing...
	ExtrinsicCalEntry(
		const double image_px[2],

		const double BASE_to_TIP_r[9], const double BASE_to_TIP_t[3],
		
		const double TARGET_to_POINT_r[9], const double TARGET_to_POINT_t[3]
	){
		image_pixels[0] = image_px[0];
		image_pixels[1] = image_px[1];
		
		for(int i = 0; i < 9; i++){
			BASE_to_TIP_rotation[i] = BASE_to_TIP_r[i];
			TARGET_to_POINT_rotation[i] = TARGET_to_POINT_r[i];
		}
		BASE_to_TIP_translation[0] = BASE_to_TIP_t[0];
		BASE_to_TIP_translation[1] = BASE_to_TIP_t[1];
		BASE_to_TIP_translation[2] = BASE_to_TIP_t[2];
		TARGET_to_POINT_translation[0] = TARGET_to_POINT_t[0];
		TARGET_to_POINT_translation[1] = TARGET_to_POINT_t[1];
		TARGET_to_POINT_translation[2] = TARGET_to_POINT_t[2];
	}

	//Takes the per-point constants.
	static ceres::CostFunction* Create(
		const double image_px[2],
		const double BASE_to_TIP_r[9], const double BASE_to_TIP_t[3],
		const double TARGET_to_POINT_r[9], const double TARGET_to_POINT_t[3]
	){
		return new ceres::AutoDiffCostFunction<ExtrinsicCalEntry, 2,
			3, 3, 3, 3, 4, 5
		>(new ExtrinsicCalEntry(
			image_px,
			
			BASE_to_TIP_r, BASE_to_TIP_t,
			TARGET_to_POINT_r, TARGET_to_POINT_t
		));
	}
};

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	// Storage for input values constant for each point.
	std::vector<Eigen::Affine3d> BASE_to_TIP_vec;
	std::vector<Eigen::Affine3d> TARGET_to_POINT_vec;
	std::vector<Eigen::Vector2d> image_pixel_vec;
	
	//read the calibration file:
	//TODO Parametrize dis
	
	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	std::string fname(data_path + "/intermediate/intrinsic_calibration_points.csv");
	if (!read_calibration_file(fname, BASE_to_TIP_vec, TARGET_to_POINT_vec, image_pixel_vec)) {
		cout<<"could not open file "<<fname<<"; quitting"<<endl;
		return 1;
	}
	cout<<"calibration file has been read in"<<endl;
	
	//Some random values that get things not just 0.
	std::cout << "BASE_to_TIP\n" << BASE_to_TIP_vec[164].matrix() << "\n\n";
	std::cout << "TARGET_to_POINT\n" << TARGET_to_POINT_vec[64].matrix() << "\n\n";
	
	// Build the problem.
	Problem problem;
	int nlines = (int) image_pixel_vec.size();
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).

	//Known values (one per data point)
	double image_u,image_v;
	double BASE_to_TIP [16];
	double TARGET_to_POINT [16];
	//Known values (constant)
	//There are NO known values which are constant for this problem!

	//Unknown values
	double TIP_to_TARGET_t [3];
	double TIP_to_TARGET_r [3];
	double CAM_to_BASE_t [3];
	double CAM_to_BASE_r [3];
	double projection_matrix[12] = {
		434.8289178042981,	0.0,			1152.0,	0.0,
		0.0,			434.8289178042981,	648.0,	0.0,
		0.0,			0.0,			1.0,	0.0
	};
	double projection[4] = {
		projection_matrix[0], projection_matrix[2], projection_matrix[5], projection_matrix[6]
	};
	double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	
	//Initialize the unknowns
	//TODO Find a way to inform this parametrically
	//TODO For simulation tests, get these ground truths from the simulation
	TIP_to_TARGET_t[0] = 0.0;
	TIP_to_TARGET_t[1] = 0.0;
	TIP_to_TARGET_t[2] = 0.0;
	TIP_to_TARGET_r[0] = 0.;
	TIP_to_TARGET_r[1] = 0.;
	TIP_to_TARGET_r[2] = 0.;
	CAM_to_BASE_t[0] = -0.3;
	CAM_to_BASE_t[1] = -0.15;
	CAM_to_BASE_t[2] = 0.3;
	CAM_to_BASE_r[0] = 0.;
	CAM_to_BASE_r[1] = 0.;
	CAM_to_BASE_r[2] = 0.;
	
	for (int i=0; i<nlines; i++) {//For each data entry...
		double pixvec[2] = {image_pixel_vec[i][0], image_pixel_vec[i][1]};
	
		//Add the per-point constants
		Eigen::Vector3d translation_t = TARGET_to_POINT_vec[i].translation();

		//TODO- Target to point actually has no need for a rotation. Remove it.
		Eigen::Matrix3d matrix_t = TARGET_to_POINT_vec[i].linear();
		double spatial_t[3] = {
			translation_t.x(),
			translation_t.y(),
			translation_t.z()
		};
		double rotation_t[9] = {
			matrix_t(0,0), matrix_t(0,1), matrix_t(0,2),
			matrix_t(1,0), matrix_t(1,1), matrix_t(1,2),
			matrix_t(2,0), matrix_t(2,1), matrix_t(2,2)
		};
		
		Eigen::Vector3d translation_b = BASE_to_TIP_vec[i].translation();
		Eigen::Matrix3d matrix_b = BASE_to_TIP_vec[i].linear();
		double spatial_b[3] = {
			translation_b.x(),
			translation_b.y(),
			translation_b.z()
		};
		double rotation_b[9] = {
			matrix_b(0,0), matrix_b(0,1), matrix_b(0,2),
			matrix_b(1,0), matrix_b(1,1), matrix_b(1,2),
			matrix_b(2,0), matrix_b(2,1), matrix_b(2,2)
		};

		CostFunction *cost_function = ExtrinsicCalEntry::Create(
			pixvec,
			rotation_b, spatial_b,
			rotation_t, spatial_t
		);
		
		//Add the parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			TIP_to_TARGET_t, TIP_to_TARGET_r,
			CAM_to_BASE_t, CAM_to_BASE_r,
			projection, distortion
		);
	}
	
	
	//Bound the rotations.
	problem.SetParameterLowerBound(TIP_to_TARGET_r, 0, -180.0);
	problem.SetParameterUpperBound(TIP_to_TARGET_r, 0,  180.0);
	problem.SetParameterLowerBound(TIP_to_TARGET_r, 1, -180.0);
	problem.SetParameterUpperBound(TIP_to_TARGET_r, 1,  180.0);
	problem.SetParameterLowerBound(TIP_to_TARGET_r, 2, -180.0);
	problem.SetParameterUpperBound(TIP_to_TARGET_r, 2,  180.0);
	
	
	problem.SetParameterLowerBound(CAM_to_BASE_r, 0, -180.0);
	problem.SetParameterUpperBound(CAM_to_BASE_r, 0,  180.0);
	problem.SetParameterLowerBound(CAM_to_BASE_r, 1, -180.0);
	problem.SetParameterUpperBound(CAM_to_BASE_r, 1,  180.0);
	problem.SetParameterLowerBound(CAM_to_BASE_r, 2, -180.0);
	problem.SetParameterUpperBound(CAM_to_BASE_r, 2,  180.0);
	
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

	std::printf("TIP_to_TARGET:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", TIP_to_TARGET_t[0], TIP_to_TARGET_t[1], TIP_to_TARGET_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(TIP_to_TARGET_r[0]), dtor(TIP_to_TARGET_r[1]), dtor(TIP_to_TARGET_r[2]));
	
	std::printf("CAM_to_BASE:\n");
	std::printf("\tx = %f\ty = %f\tz = %f\n", CAM_to_BASE_t[0], CAM_to_BASE_t[1], CAM_to_BASE_t[2]);
	std::printf("\tr = %f\tp = %f\tw = %f\n", dtor(CAM_to_BASE_r[0]), dtor(CAM_to_BASE_r[1]), dtor(CAM_to_BASE_r[2]));
	
	std::printf("INTRINSICS:\n");
	std::printf(
		"\tfx = %f\tfy = %f\t cx = %f\t cy = %f\n",
		projection[0], projection[1], projection[2], projection[3]
	);
	std::printf(
		"\tk1 = %f\tk2 = %f\t k3 = %f\t p1 = %f\t p2 = %f\n",
		distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]
	);

	return 0;
}
