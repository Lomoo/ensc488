#ifndef globalRobotCofig_h__
#define globalRobotCofig_h__


#include <windows.h>
#include <fstream>



//////////////////////////
// Trajectory Constants //
//////////////////////////
#define filename "viapoints.txt"
//#define filename "velocityLimits.txt"
//#define filename "JointLimits.txt"
//#define filename "JointLimits2.txt"
//#define filename "extraTests.txt"
#define SAMPLING_RATE 10
#define MAX_VIA_POINTS 5
#define MAX_TIME 17

#define S_TO_MILIS 1000.0

#define SAMPLING_RATE_T1 SAMPLING_RATE
#define SAMPLING_RATE_T2 (SAMPLING_RATE_T1*10)
#define SAMPLING_RATE_T3 (SAMPLING_RATE_T2*10)

#define DELTA_T1 ((double)1.0/SAMPLING_RATE_T1)
#define DELTA_T2 ((double)1.0/SAMPLING_RATE_T2)
#define DELTA_T3 ((double)1.0/SAMPLING_RATE_T3)

#define MAX_DATA_POINTS ((int)SAMPLING_RATE_T1 * MAX_TIME)



/////////////////////////////////////
// Global Constants and Parameters //
/////////////////////////////////////


////////////////////
// Link Constants //
////////////////////
const double L1 = 405.0;
const double L2 = 70.0;
const double L3 = 195.0;
const double L4 = 142.0;
const double L5 = 410.0;
const double L6 = 80.0;
const double L7 = 60.0;
const double L8 = 30.0;
const int NUM_OF_LINK_VARS = 4;
const double m1 = 1.7;
const double m2 = 1.0;
const double m3 = 1.7;
const double m4 = 1.0;
const double g = 9.81 * 1000;
const double FrictionCoef = 5.0;


//////////////////
// Angle Limits //
//////////////////
const double THETA1_MAX = 150.0f;
const double THETA1_MIN = -150.0f;
const double VelTheta1_MAX = 150.0f;
const double AccTheta1_Max = 600.0f;

const double THETA2_MAX = 100.0f;
const double THETA2_MIN = -100.0f;
const double VelTheta2_MAX = 150.0f;
const double AccTheta2_Max = 600.0f;

const double D3_MAX = -100.0f;
const double D3_MIN = -200.0f;
const double VelD3_MAX = 50.0f;
const double AccD3_Max = 200.0f;

const double THETA4_MAX = 160.0f;
const double THETA4_MIN = -160.0f;
const double VelTheta4_MAX = 150.0f;
const double AccTheta4_Max = 600.0f;

//////////////////
// Known Frames //
//////////////////
const matrix T_SB = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
const matrix T_WT = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, L7 }, { 0, 0, 0, 1 } };





// Limit Check Functions //
bool Theta1Check(double ang)
{
	return ((ang > THETA1_MAX || ang < THETA1_MIN) ? false : true);
}

bool Theta2Check(double ang)
{
	return ((ang > THETA2_MAX || ang < THETA2_MIN) ? false : true);
}

bool Theta4Check(double ang)
{
	return ((ang > THETA4_MAX || ang < THETA4_MIN) ? false : true);
}

bool D3Check(double dist)
{
	return ((dist > D3_MAX || dist < D3_MIN) ? false : true);
}

bool Theta1Check2(double ang)
{
	return ((ang >= THETA1_MAX || ang <= THETA1_MIN) ? false : true);
}

bool Theta2Check2(double ang)
{
	return ((ang >= THETA2_MAX || ang <= THETA2_MIN) ? false : true);
}

bool Theta4Check2(double ang)
{
	return ((ang >= THETA4_MAX || ang <= THETA4_MIN) ? false : true);
}

bool D3Check2(double dist)
{
	return ((dist <= D3_MAX || dist >= D3_MIN) ? false : true);
}

//Velocity
bool VelTheta1Check(double vel)
{
	if ((vel > VelTheta1_MAX || vel < -VelTheta1_MAX)) {
		return true;
	}
	else {
		return false;
	}
}

bool VelTheta2Check(double vel)
{
	if ((vel > VelTheta2_MAX || vel < -VelTheta2_MAX)) {
		return true;
	}
	else {
		return false;
	}
}

bool VelD3Check(double vel)
{
	if ((vel > VelD3_MAX || vel < -VelD3_MAX)) {
		return true;
	}
	else {
		return false;
	}
}

bool VelTheta4Check(double vel)
{
	if ((vel > VelTheta4_MAX || vel < -VelTheta4_MAX)) {
		return true;
	}
	else {
		return false;
	}
}

//Accleration
bool AccTheta1Check(double acc)
{
	if ((acc > AccTheta4_Max || acc < -AccTheta4_Max)) {
		return true;
	}
	else {
		return false;
	}
}

bool AccTheta2Check(double acc)
{
	if ((acc > AccTheta4_Max || acc < -AccTheta4_Max)) {
		return true;
	}
	else {
		return false;
	}
}

bool AccD3Check(double acc)
{
	if ((acc > AccD3_Max || acc < -AccD3_Max)) {
		return true;
	}
	else {
		return false;
	}
}

bool AccTheta4Check(double acc)
{
	if ((acc > AccTheta4_Max || acc < -AccTheta4_Max)) {
		return true;
	}
	else {
		return false;
	}
}
void JointToVect(JOINT joint, vect& vector)
{
	for (int i = 0; i < NUM_OF_LINK_VARS; i++)
	{
		vector[i] = joint[i];
	}
}

void VectToJoint(vect vector, JOINT& joint)
{
	for (int i = 0; i < NUM_OF_LINK_VARS; i++)
	{
		joint[i] = vector[i];
	}
}

void GetCurrentConfig(vect& curConfig)
{
	JOINT Config;
	GetConfiguration(Config);
	VectorCopy(Config, curConfig);
}

/////////////////////
// Timer Functions //
/////////////////////

double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		std::cout << "QueryPerformanceFrequency failed!\n";

	PCFreq = double(li.QuadPart) / S_TO_MILIS;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
}

double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart - CounterStart) / PCFreq;
}

void microsleep(double duration)
{
	StartCounter();
	while (true)
	{
		if (GetCounter() >= duration)
		{
			return;
		}
	}

}

////////////////////////
// File Writing Stuff //
////////////////////////

std::ofstream OpenFile(std::string file_name)
{
	std::ofstream fid;
	fid.open(file_name, std::ofstream::out | std::ofstream::trunc);
	if (!fid.is_open())
	{
		std::cout << "Error: Cannot open file " << file_name.c_str() << std::endl;
		throw std::exception("File Could not open!");
	}
	return fid;
}

//Save sampled trajectory into txt file
void Write2File(std::ofstream &fid, double time, vect out_vect)
{
	if (fid.is_open()) {
		fid << time << " ";
		for (int j = 0; j < VECTOR_SIZE; j++) {
			fid << out_vect[j] << " ";
		}
		fid << std::endl;
	}
}

void Write2File(std::ofstream &fid, vect CartCoords)
{
	if (fid.is_open()) {
		for (int j = 0; j < 2; j++) {
			fid << CartCoords[j] << " ";
		}
		fid << std::endl;
	}
}

void CloseFile(std::ofstream &fid)
{
	fid.close();
}

#endif // globalRobotCofig_h__