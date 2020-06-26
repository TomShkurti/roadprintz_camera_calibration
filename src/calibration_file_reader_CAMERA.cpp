// module to read a calibration file stored line-by-line in CSV file format

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator>>(istream& ins, record_t& record) {
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	string line;
	getline(ins, line);

	// now we'll use a stringstream to separate the fields out of the line
	stringstream ss(line);
	string field;
	while (getline(ss, field, ',')) {
		// for each field we wish to convert it to a double
		// (since we require that the CSV contains nothing but floating-point values)
		stringstream fs(field);
		double f = 0.0; // (default value is 0.0)
		fs >> f;

		// add the newly-converted field to the end of the record
		record.push_back(f);
	}

	// Now we have read a single line, converted into a list of fields, converted the fields
	// from strings to doubles, and stored the results in the argument record, so
	// we just return the argument stream as required for this kind of input overload function.
	return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator>>(istream& ins, data_t& data) {
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record) {
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}

//return of "true" means all is well
bool read_calibration_file(
	std::string fname,
	std::vector<Eigen::Affine3d> &BASE_to_POINT_vec,
	std::vector<Eigen::Affine3d> &FOREARM_to_BASE_vec,
	std::vector<Eigen::Vector2d> &image_pixel_vec
) {
	
		//open the trajectory file:
	ifstream infile(fname.c_str());
	if (!infile){ //The file couldn't be opened.
		cerr << "Error: file could not be opened; giving up" << endl;
		return false;
	}
	

	// Here is the data we want.
	data_t data;

	// Here is the file containing the data. Read it into data.
	infile >> data;

	// Complain if something went wrong.
	if (!infile.eof()) {
		cout << "error reading file!\n";
		return false;
	}

	infile.close();

	// Otherwise, list some basic information about the file.
	cout << "CSV file contains " << data.size() << " records.\n";

	unsigned min_record_size = data[0].size();
	unsigned max_record_size = 0;
	for (unsigned n = 0; n < data.size(); n++) {
		if (max_record_size < data[ n ].size())
			max_record_size = data[ n ].size();
		if (min_record_size > data[ n ].size())
			min_record_size = data[ n ].size();
	}
	if (max_record_size > 34) {
		cout<<"bad file"<<endl;
		cout << "The largest record has " << max_record_size << " fields.\n";
		return false;
	}
	if (min_record_size < 34) {
		cout<<"bad file"<<endl;
		cout << "The smallest record has " << min_record_size << " fields.\n";
		return false;
	}

	
	//the following args are passed in. Make sure they are empty
	BASE_to_POINT_vec.clear();
	FOREARM_to_BASE_vec.clear();
	image_pixel_vec.clear();
	
        Eigen::Vector2d xy_pixels;
        Eigen::Affine3d BASE_to_POINT;
	Eigen::Affine3d FOREARM_to_BASE;
	
	int nlines = data.size();
	for (int n = 0; n < nlines; n++) {
		Eigen::Matrix4d m;
		m <<
			data[n][ 0], data[n][ 1], data[n][ 2], data[n][ 3],
			data[n][ 4], data[n][ 5], data[n][ 6], data[n][ 7],
			data[n][ 8], data[n][ 9], data[n][10], data[n][11],
			data[n][12], data[n][13], data[n][14], data[n][15]
		;
		BASE_to_POINT = m.transpose();
		m <<
			data[n][16], data[n][17], data[n][18], data[n][19],
			data[n][20], data[n][21], data[n][22], data[n][23],
			data[n][24], data[n][25], data[n][26], data[n][27],
			data[n][28], data[n][29], data[n][30], data[n][31]
		;
		FOREARM_to_BASE = m;
		
		xy_pixels(0) = data[n][32];
		xy_pixels(1) = data[n][33];
        
		image_pixel_vec.push_back(xy_pixels);
		BASE_to_POINT_vec.push_back(BASE_to_POINT);
		FOREARM_to_BASE_vec.push_back(FOREARM_to_BASE);
	}
	return true;
}