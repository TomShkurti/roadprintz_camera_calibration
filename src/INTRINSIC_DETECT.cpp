//test findCountours; building up to camera calibration
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//TODO Why are these deps listed twice?
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include <boost/filesystem.hpp>

#include <roadprintz_camera_calibration/circle_detector.hpp>

//WOULD NEED TO CHANGE THESE FOR SPECIFIC PATTERN
//TODO These should really go in a yml file or a ros param or something.
const double CIRCLE_SPACING = 0.03556;//spacing of circle array: est 1.4"
const int N_CIRCLE_ROWS = 5;
const int N_CIRCLE_COLS = 5;
const double MILL_INCREMENT = 0.1; //moved sled by 0.1m each image



#define DEBUG_CIRCLE_DETECTOR

//const int w = 500;
int levels = 6; //5;
int g_width;
int g_height;

int THRESH = 150;
int g_ans;

/*vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

vector < Mat> g_vec_of_images;
vector <string> g_vec_of_image_names;
int g_n_images;
int g_imagenum=0;

struct CV_EXPORTS Center
{
Point2d location;
double radius;
double confidence;
};*/

int read_images(
	const std::string & folder,
	std::vector<cv::Mat> & vec_of_images,
	std::vector<std::string> & vec_of_image_names
){
	boost::filesystem::path image_folder_path = boost::filesystem::path(folder);
	for(
		boost::filesystem::directory_entry& x
		:
		boost::filesystem::directory_iterator(image_folder_path)
	){
		vec_of_image_names.push_back(x.path().native());
		cv::Mat im = cv::imread(x.path().native(), CV_LOAD_IMAGE_COLOR);
		vec_of_images.push_back(im);
	}
	printf("Loaded %lu images.\n", vec_of_image_names.size());
	return vec_of_image_names.size();
}

/*typedef struct
{
int pattern_rows;// number of rows
int pattern_cols;// number of colulmns
bool is_symmetric; // not sure
double circle_diameter;// size of each circle
double spacing;// spacing between circle centers
} CircleGridParameters;

struct Params {
int thresholdStep = 10;
int minThreshold = 50;
int maxThreshold = 220;
int minRepeatability = 2;
int minDistBetweenCircles = 10;
int minRadiusDiff = 10;

bool filterByColor = true;
int circleColor = 0;

 bool filterByArea = true;
int minArea = 25; //25;//int
int maxArea = 50000;//int

bool filterByCircularity = false;
double minCircularity = 0;//double
double maxCircularity = 1;

bool filterByInertia = true;
int minInertiaRatio = 0.1; //double
int maxInertiaRatio = 1;

bool filterByConvexity = false;
double minConvexity = 0.95;
double maxConvexity = 1;
};
Params params;
/*
thresholdStep = 10;
minThreshold = 50;
maxThreshold = 220;
minRepeatability = 2;
minDistBetweenCircles = 10;
minRadiusDiff = 10;

filterByColor = true;
circleColor = 0;

filterByArea = true;
minArea = 25;//int
maxArea = 5000;//int

filterByCircularity = false;
minCircularity = 0.8f;//double
maxCircularity = std::numeric_limits<float>::max();

filterByInertia = true;
minInertiaRatio = 0.1f; //double
maxInertiaRatio = std::numeric_limits<float>::max();

filterByConvexity = true;
minConvexity = 0.95f;
maxConvexity = std::numeric_limits<float>::max(); 


bool filterByArea = true;
intminArea = 25;//int
intmaxArea = 5000;//int

boolfilterByColor = true;
intcircleColor = 0;

bool filterByInertia = true;
double minInertiaRatio = 0.1;
double maxInertiaRatio = 10;

bool filterByConvexity = true;
doubleminConvexity = 0;
doublemaxConvexity = 0; 
 */
//Mat g_cnt_img;

//void CircleDetectorImpl::findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) const

/*void findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) 
{
//CV_INSTRUMENT_REGION()

Mat image = _image.getMat();// Oh so muchcleaner this way :(
Mat binaryImage = _binaryImage.getMat();

(void)image;
centers.clear();

vector<vector<Point> > contours;
Mat tmpBinaryImage = binaryImage.clone();
findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

// loop on all contours
for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
{
// each if statement may eliminate a contour through the continue function
// some if statements may also set the confidence whose default is 1.0
Center center;
center.confidence = 1;
Moments moms = moments(Mat(contours[contourIdx]));

if (params.filterByArea)
{
double area = moms.m00;
if (area < params.minArea || area >= params.maxArea) continue;
}

if (params.filterByCircularity)
{
double area = moms.m00;
double perimeter = arcLength(Mat(contours[contourIdx]), true);
double ratio = 4 * CV_PI * area / (perimeter * perimeter);
if (ratio < params.minCircularity || ratio >= params.maxCircularity) continue;
}

if (params.filterByInertia)
{
double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
const double eps = 1e-2;
double ratio;
if (denominator > eps)
{
double cosmin = (moms.mu20 - moms.mu02) / denominator;
double sinmin = 2 * moms.mu11 / denominator;
double cosmax = -cosmin;
double sinmax = -sinmin;

double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
ratio = imin / imax;
}
else
{
ratio = 1;
}

if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) continue;

center.confidence = ratio * ratio;
}

if (params.filterByConvexity)
{
vector<Point> hull;
convexHull(Mat(contours[contourIdx]), hull);
double area = contourArea(Mat(contours[contourIdx]));
double hullArea = contourArea(Mat(hull));
double ratio = area / hullArea;
if (ratio < params.minConvexity || ratio >= params.maxConvexity) continue;
}
Mat pointsf;
Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
if (pointsf.rows < 5) continue;
RotatedRect box = fitEllipse(pointsf);

// find center
// center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);
center.location = box.center;

// one more filter by color of central pixel
if (params.filterByColor)
{
if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor) continue;
}

// compute circle radius
//	{
//	vector<double> dists;
//	for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
//	{
//		Point2d pt = contours[contourIdx][pointIdx];
//		dists.push_back(norm(center.location - pt));
//	}
//	std::sort(dists.begin(), dists.end());
//	center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
//}
center.radius = (box.size.height + box.size.width) / 4.0;
centers.push_back(center);

}
}


//This has to do with the "TODO: This code does nothing" issue in Main.
/*
//note: required level 5,6 or 7 to see contours of circles
static void on_trackbar(int, void*)
{
Mat cnt_img = Mat::zeros(g_height,g_width,CV_8UC3);
//levels=5 seems good--finds circle contours at min of tiny contours
cout<<"levels = "<<levels<<endl;
int _levels = levels - 3;
drawContours( cnt_img, contours, _levels <= 0 ? 3 : -1, Scalar(128,255,255),
3, LINE_AA, hierarchy, std::abs(_levels) );


imshow("contours", cnt_img);
g_cnt_img = cnt_img;
}

//This has to do with the "TODO: This code does nothing" issue in Main.
/*static void on_imagebar(int, void*)
{
Mat image = g_vec_of_images[g_imagenum];
cout<<"g_imagenum = "<<g_imagenum<<endl;
imshow("images", image);
}*/


//the next fnc infers displacements from names of files;
//this is specialized for names of the form image_3_2_1.jpg, where
// "3" means 3*0.1m of x travel, "2" is 2*0.1m of y travel and "1" --> 1*0.1m of z travel;
// this function must convert mill displacements into target displacements, w/ mill frame approx aligned
// with target frame.I.e. target x-axis is left-to-right in image, consistent w/ camera frame;
//target y-axis is top-to-bottom, also consistent w/ camera frame, 
//and target z-axis is closest-to-furthest, also consistent w/ camera frame
// CHANGE mill recordings, for which (in this specific instance):
//x values varied from far right to far left, increments of 100mm
//y values varied from closest to furthest, increments of 100mm
//z values varied from lowest to highest, increments of 100mm
//reinterpret these: (approx) target-frame x-translation = -x_index*0.1m
// approx target-frame y translation = -z_index*0.1m
// approx target-frame z translation = y_index*0.1m


void get_dxdydz_from_image_name(
	const std::string& image_name,
	double &dx_mill,
	double &dy_mill,
	double &dz_mill
) {
	//TODO the following is not very flexible: extract dx,dy,dz indices from file name, SINGLE DIGIT
	std::string image_name_last = image_name.substr(image_name.find_last_of("/") + 1);
	std::string s_index = image_name_last.substr(6,1); //str.substr (3,5);
	int x_index = stoi(s_index);
	s_index = image_name_last.substr(8,1);
	int y_index = stoi(s_index);
	s_index = image_name_last.substr(10,1);
	int z_index = stoi(s_index);
	
	//convert indices into translations:
	dx_mill = -x_index*MILL_INCREMENT;
	dy_mill = -z_index*MILL_INCREMENT; //mill z-axis is antiparallel to target y-axis
	dz_mill = y_index*MILL_INCREMENT; //mill y-axis is in direction of camera/target z-axis
	//cout<<"image name "<<image_name<<" implies indices "<<x_index<<", "<<y_index<<", "<<z_index<<endl;
	//cout<<"inferred displacements, approx aligned w/ target frame: "<<dx_mill<<", "<<dy_mill<<", "<<dz_mill<<endl;
	//cout<<"enter 1: ";
	//cin>>g_ans; 
}


int main(int argc, char** argv) {
	//ros::init(argc, argv, "contour_finder");
	//ros::NodeHandle n; //

	/*if( argc != 2){
		cout <<" Usage: rosrun example_opencv test_find_contours ImageToLoadAndDisplay" << endl;
		return -1;
	}*/
	
	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	std::string intermediate_folder = data_path + "/intermediate";
	//Start up the file where outputs will go.
	std::ofstream calib_output_file;
	calib_output_file.open(intermediate_folder + "/intrinsic_calibration_points.csv");
	
	std::vector<cv::Mat> vec_of_images;
	std::vector<std::string> vec_of_image_names;
	int n_images = read_images(data_path + "/images", vec_of_images, vec_of_image_names);
	//Check and get data
	for(int i = 0; i < n_images; i++){
		cv::Mat example_image = vec_of_images[i];
		if(! example_image.data ){// Check for invalid input
			printf("Could not open or find image %s\n", vec_of_image_names[i].c_str());
			return -1;
		}
	}
	int width = vec_of_images[0].cols;
	int height = vec_of_images[0].rows;
	printf("Width: %d\n", width);
	printf("Height: %d\n", height);

	//TODO As near as I can determine, this section of code in fact does nothing.
	//	It shows a sample image and allows the user to tune the thresholding value
	//	until the circles are subjectively good-n-visible. However, the threshold
	//	values it saves are REPLACED later on in the program and thus this
	//	manual calibration step is not actually useful.
	//
	//	Find out if this step is helpful enough to be put back into the
	//	production code. If it is, clean it up a little. Otherwise remove
	//	it.
	/*
	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "Display window", image ); // Show our image inside it.//convert image to binary:
	Mat grayscaleImage;
	cvtColor(image, grayscaleImage, CV_BGR2GRAY);

	Mat binarizedImage;
	double thresh=THRESH;
	cout<<"enter thresh (e.g. 125): ";
	//cin>>thresh;//looked OK at 100 and 150; NOT 50or 200; try 125
	threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

	namedWindow( "binary", 1 );
	imshow( "binary", binarizedImage );

	vector<vector<Point> > contours0;
	findContours( binarizedImage, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	contours.resize(contours0.size());
	double eps = 0.1;
	for( size_t k = 0; k < contours0.size(); k++ ) 
	approxPolyDP(Mat(contours0[k]), contours[k], eps, true);

	namedWindow( "contours", 1 );
	namedWindow( "images", 1 );
	//levels=5;

	createTrackbar( "levels+3", "contours", &levels, 7, on_trackbar );
	createTrackbar( "image_num", "images", &g_imagenum, g_n_images-1, on_imagebar );


	on_trackbar(0,0); 
	on_imagebar(0,0);
	cout<<"finding circles..."<<endl;
	//vector<Center> curCenters;
	//findCircles(grayscaleImage, binarizedImage, curCenters);

	//find a 5x5 grid of circles
	//Size patternsize(5,5); //number of centers*/
	cv::Size patternsize(5,5); //TODO Parametrize this

	std::vector<cv::Point2f> centers; //this will be filled by the detected centers
	// Set up the detector with default parameters.
	cv::CircleDetector circleDetector; 

	/**
	*@brief circle_detector_ptr_ is a custom blob detector which localizes circles better than simple blob detection
	*/
	cv::Ptr<cv::CircleDetector> circle_detector_ptr_ = cv::CircleDetector::create();
	
	cv::namedWindow("Src image");

 	printf("Processing %d images.\n", n_images);
	int failures=0;
	for (int i_image = 0; i_image < n_images; i_image++){
		cv::Mat image;
		cv::Mat grayscaleImage;
		cv::Mat binarizedImage;
		
		image = vec_of_images[i_image];
		printf("i_image = %d\n", i_image);
		cv::Point2f center; // = centers[i];
		cv::cvtColor(image, grayscaleImage, CV_BGR2GRAY);

		double thresh=THRESH;
		//TODO This odd bit of nothing-code again.
		//cout<<"enter thresh (e.g. 125): ";
		//cin>>thresh;//looked OK at 100 and 150; NOT 50or 200; try 125
		//TODO This is never used?
		cv::threshold(grayscaleImage, binarizedImage, thresh, 255, 0); 
		
		//TODO Looks like a bunch of different tweaks to the same circle-finder.
		/**/
		//bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_CLUSTERING);//CALIB_CB_CLUSTERING
		//bool patternfound = findCirclesGrid(binarizedImage, patternsize, centers, cv::CALIB_CB_SYMMETRIC_GRID);//CALIB_CB_CLUSTERING
		//bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID);//CALIB_CB_CLUSTERING
		//bool patternfound = findCirclesGrid(binarizedImage, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);//CALIB_CB_CLUSTERING
		//bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID,blobDetector);//CALIB_CB_CLUSTERING
 		//successful_find = cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID,
		// circle_detector_ptr_);
		//xxx wsn test:
		//virtual void findCircles(InputArray image, InputArray binaryImage, std::vector<Center>& centers)
		//std::vector<Center> vec_of_Centers;
		//std::vector<cv::CircleDetectorImpl::Center> vec_of_Centers;
		//cv::CircleDetectorImpl circleDetectorImpl;
		//cv::CircleDetectorImpl circleDetectorImpl;

		//circleDetectorImpl.findCircles(grayscaleImage,binarizedImage,vec_of_Centers );
		//circleDetectorImpl.findCircles(grayscaleImage,binarizedImage,vec_of_Centers );

		bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, 1, circle_detector_ptr_);//CALIB_CB_CLUSTERING
		double dx_mill,dy_mill,dz_mill;
 
		if (patternfound) {
			printf("\tPattern found in image %d\n", i_image);
			get_dxdydz_from_image_name(vec_of_image_names[i_image],dx_mill,dy_mill,dz_mill);
			int ncircles = centers.size();
			printf("\tfound %d circles\n", ncircles);
			
			for (int i_circle=0; i_circle<N_CIRCLE_COLS; i_circle++){
				for (int j_circle=0; j_circle<N_CIRCLE_ROWS; j_circle++) {
					int n_circle = j_circle + i_circle*N_CIRCLE_COLS;
					center = centers[n_circle];
					cv::circle( image, center, 2, cv::Scalar(0,0,255), 2 );
					calib_output_file <<
						"1,0,0,0,0,1,0,0,0,0,1,0," <<
						dx_mill << ", " << dy_mill <<
						", " << dz_mill << ", 1,\t" <<
						
						"1,0,0,0,0,1,0,0,0,0,1,0," <<//TODO Make doubly sure the target scaling is fixed!!!
						j_circle*CIRCLE_SPACING * 0.98 << ", " <<
						i_circle*CIRCLE_SPACING * 0.98<< ", " <<
						"0.01, 1,\t" << 
						
						center.x << ", " <<
						center.y << "\n";
					;
				}
			}
		} else {
			printf("\tpattern not found in image %d\n", i_image);
			
			failures++;
		}
		cv::imshow("Src image", image);
		cv::waitKey(1000);
	}
	calib_output_file.close();
	return 0;
}
