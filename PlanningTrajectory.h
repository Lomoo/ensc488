#ifndef PlanningTajectory_h__
#define PlanningTajectory_h__

#include "matrix.h"
#include "globalRobotConfig.h"
#include <exception>
#include <cmath>
#include "InverseKin.h"
#include <fstream>



//Read Via points from text file "viapoints"
void ReadViaPoints(double via_times[5], double x_via[5], double y_via[5], double z_via[5], double phi_via[5], int num_via) {
	double temp[5][5];
	std::ifstream in(filename);

	if (!in) {
		std::cout << "Cannot open file.\n";
		return;
	}

	for (int line = 0; line < num_via; line++) {
		for (int row = 0; row < 5; row++) {
			in >> temp[line][row];
		}
	}
	in.close();
	for (int i = 0; i < num_via; i++) {
		via_times[i] = temp[i][0];
	}


	for (int i = 0; i < num_via; i++) {
		x_via[i] = temp[i][1];
	}
	for (int i = 0; i < num_via; i++) {
		y_via[i] = temp[i][2];
	}
	for (int i = 0; i < num_via; i++) {
		z_via[i] = temp[i][3];
	}
	for (int i = 0; i < num_via; i++) {
		phi_via[i] = temp[i][4];
	}
}

// Generate Trajectory Parameters
void TraGen(double via_times[5], double theta1_via[5], double theta2_via[5], double d3_via[5], double theta4_via[5], matrix& param1, matrix& param2, matrix& param3, matrix& param4, int num_via) {




	double h[4];
	for (int i = 0; i < 4; i++) {
		h[i] = via_times[i + 1] - via_times[i];
	}


	//via pt velocities
	double v1_via[MAX_VIA_POINTS];
	double v2_via[MAX_VIA_POINTS];
	double v3_via[MAX_VIA_POINTS];
	double v4_via[MAX_VIA_POINTS];

	for (int i = 0; i < num_via; i++) {
		v1_via[i] = 0;
		v2_via[i] = 0;
		v3_via[i] = 0;
		v4_via[i] = 0;

	}

	//calculate via point velocities as average//
	//inital and final velocity remain 0 to obtain a smooth start/stop

	for (int i = 1; i < num_via - 1; i++) {
		v1_via[i] = (((theta1_via[i] - theta1_via[i - 1]) / h[i - 1]) + ((theta1_via[i + 1] - theta1_via[i]) / h[i])) / 2;
	}
	for (int i = 1; i < num_via - 1; i++) {
		v2_via[i] = (((theta2_via[i] - theta2_via[i - 1]) / h[i - 1]) + ((theta2_via[i + 1] - theta2_via[i]) / h[i])) / 2;
	}
	for (int i = 1; i < num_via - 1; i++) {
		v3_via[i] = (((d3_via[i] - d3_via[i - 1]) / h[i - 1]) + ((d3_via[i + 1] - d3_via[i]) / h[i])) / 2;
	}
	for (int i = 1; i < num_via - 1; i++) {
		v4_via[i] = (((theta4_via[i] - theta4_via[i - 1]) / h[i - 1]) + ((theta4_via[i + 1] - theta4_via[i]) / h[i])) / 2;
	}

	//calculate parameters//

	// for theta1:
	for (int i = 0; i < num_via - 1; i++) {
		param1[i][0] = theta1_via[i];  //parameter a
	}
	for (int i = 0; i < num_via - 1; i++) {
		param1[i][1] = h[i] * v1_via[i]; //parameter b
	}
	for (int i = 0; i < num_via - 1; i++) {
		param1[i][3] = h[i] * v1_via[i + 1] + 2 * param1[i][0] + param1[i][1] - 2 * theta1_via[i + 1]; //parameter d
	}
	for (int i = 0; i < num_via - 1; i++) {
		param1[i][2] = theta1_via[i + 1] - param1[i][0] - param1[i][1] - param1[i][3]; //parameter c
	}


	//for theta2:
	for (int i = 0; i < num_via - 1; i++) {
		param2[i][0] = theta2_via[i];  //parameter a
	}
	for (int i = 0; i < num_via - 1; i++) {
		param2[i][1] = h[i] * v2_via[i]; //parameter b
	}
	for (int i = 0; i < num_via - 1; i++) {
		param2[i][3] = h[i] * v2_via[i + 1] + 2 * param2[i][0] + param2[i][1] - 2 * theta2_via[i + 1]; //parameter d
	}
	for (int i = 0; i < num_via - 1; i++) {
		param2[i][2] = theta2_via[i + 1] - param2[i][0] - param2[i][1] - param2[i][3]; //parameter c
	}


	//for theta3:
	for (int i = 0; i < num_via - 1; i++) {
		param3[i][0] = d3_via[i];  //parameter a
	}
	for (int i = 0; i < num_via - 1; i++) {
		param3[i][1] = h[i] * v3_via[i]; //parameter b
	}
	for (int i = 0; i < num_via - 1; i++) {
		param3[i][3] = h[i] * v3_via[i + 1] + 2 * param3[i][0] + param3[i][1] - 2 * d3_via[i + 1]; //parameter d
	}
	for (int i = 0; i < num_via - 1; i++) {
		param3[i][2] = d3_via[i + 1] - param3[i][0] - param3[i][1] - param3[i][3]; //parameter c
	}


	//for theta4:
	for (int i = 0; i < num_via - 1; i++) {
		param4[i][0] = theta4_via[i];  //parameter a
	}
	for (int i = 0; i < num_via - 1; i++) {
		param4[i][1] = h[i] * v4_via[i]; //parameter b
	}
	for (int i = 0; i < num_via - 1; i++) {
		param4[i][3] = h[i] * v4_via[i + 1] + 2 * param4[i][0] + param4[i][1] - 2 * theta4_via[i + 1]; //parameter d
	}
	for (int i = 0; i < num_via - 1; i++) {
		param4[i][2] = theta4_via[i + 1] - param4[i][0] - param4[i][1] - param4[i][3]; //parameter c
	}
	return;
}

// Calculate Trajectory Joint Value and Velocitiy and Acceleration
void TraCalc(double via_times[5], matrix param1, matrix param2, matrix param3, matrix param4, int num_via, unsigned int sampling_rate,
	vect* JointPosArray,
	vect* JointVelArray,
	vect* JointAclArray,
	int& num_samples)
{

	double h[4];
	for (int i = 0; i < num_via - 1; i++) {
		h[i] = via_times[i + 1] - via_times[i];
	}


	vect num_seg_samples = { 0, 0, 0, 0 };
	double seg_offset[5] = { 0, 0, 0, 0, 0 };
	int num_seg = num_via - 1;
	num_samples = (via_times[num_seg] - via_times[0]) * SAMPLING_RATE_T1;

	// Calculating Sample Offsets and
	for (int i = 0; i < num_via - 1; i++)
	{
		num_seg_samples[i] = (via_times[i + 1] - via_times[i]) * SAMPLING_RATE_T1;
		seg_offset[i + 1] = seg_offset[i] + num_seg_samples[i];

		// Loop through segments
		for (int cur_seg = 0; cur_seg < num_via - 1; cur_seg++)
		{
			// Sample within segments
			for (int cur_sample = seg_offset[cur_seg]; cur_sample < seg_offset[cur_seg + 1]; cur_sample++)
			{
				// Parameterized Time
				double tau = (cur_sample - seg_offset[cur_seg]) / num_seg_samples[cur_seg];

				// Calculating Pos

				JointPosArray[cur_sample][0] = param1[cur_seg][0] + param1[cur_seg][1] * tau + param1[cur_seg][2] * pow(tau, 2) + param1[cur_seg][3] * pow(tau, 3);
				JointPosArray[cur_sample][1] = param2[cur_seg][0] + param2[cur_seg][1] * tau + param2[cur_seg][2] * pow(tau, 2) + param2[cur_seg][3] * pow(tau, 3);
				JointPosArray[cur_sample][2] = param3[cur_seg][0] + param3[cur_seg][1] * tau + param3[cur_seg][2] * pow(tau, 2) + param3[cur_seg][3] * pow(tau, 3);
				JointPosArray[cur_sample][3] = param4[cur_seg][0] + param4[cur_seg][1] * tau + param4[cur_seg][2] * pow(tau, 2) + param4[cur_seg][3] * pow(tau, 3);

				// Calculating Vel
				JointVelArray[cur_sample][0] = (1 / h[cur_seg])*(param1[cur_seg][1] + 2 * param1[cur_seg][2] * tau + 3 * param1[cur_seg][3] * pow(tau, 2));
				JointVelArray[cur_sample][1] = (1 / h[cur_seg])*(param2[cur_seg][1] + 2 * param2[cur_seg][2] * tau + 3 * param2[cur_seg][3] * pow(tau, 2));
				JointVelArray[cur_sample][2] = (1 / h[cur_seg])*(param3[cur_seg][1] + 2 * param3[cur_seg][2] * tau + 3 * param3[cur_seg][3] * pow(tau, 2));
				JointVelArray[cur_sample][3] = (1 / h[cur_seg])*(param4[cur_seg][1] + 2 * param4[cur_seg][2] * tau + 3 * param4[cur_seg][3] * pow(tau, 2));

				// Calculating Acl
				JointAclArray[cur_sample][0] = (1 / pow(h[num_via - 2], 2))*(2 * param1[num_via - 2][2] + 6 * param1[num_via - 2][3] * tau);
				JointAclArray[cur_sample][1] = (1 / pow(h[num_via - 2], 2))*(2 * param2[num_via - 2][2] + 6 * param2[num_via - 2][3] * tau);
				JointAclArray[cur_sample][2] = (1 / pow(h[num_via - 2], 2))*(2 * param3[num_via - 2][2] + 6 * param3[num_via - 2][3] * tau);
				JointAclArray[cur_sample][3] = (1 / pow(h[num_via - 2], 2))*(2 * param4[num_via - 2][2] + 6 * param4[num_via - 2][3] * tau);
			}
		}
	}

	//Calculating last sample
	//Calculating Pos
	JointPosArray[num_samples][0] = param1[num_via - 2][0] + param1[num_via - 2][1] + param1[num_via - 2][2] + param1[num_via - 2][3];
	JointPosArray[num_samples][1] = param2[num_via - 2][0] + param2[num_via - 2][1] + param2[num_via - 2][2] + param2[num_via - 2][3];
	JointPosArray[num_samples][2] = param3[num_via - 2][0] + param3[num_via - 2][1] + param3[num_via - 2][2] + param3[num_via - 2][3];
	JointPosArray[num_samples][3] = param4[num_via - 2][0] + param4[num_via - 2][1] + param4[num_via - 2][2] + param4[num_via - 2][3];

	// Calculating Vel
	JointVelArray[num_samples][0] = (1 / h[num_via - 2])*(param1[num_via - 2][1] + 2 * param1[num_via - 2][2] + 3 * param1[num_via - 2][3]);
	JointVelArray[num_samples][1] = (1 / h[num_via - 2])*(param2[num_via - 2][1] + 2 * param2[num_via - 2][2] + 3 * param2[num_via - 2][3]);
	JointVelArray[num_samples][2] = (1 / h[num_via - 2])*(param3[num_via - 2][1] + 2 * param3[num_via - 2][2] + 3 * param3[num_via - 2][3]);
	JointVelArray[num_samples][3] = (1 / h[num_via - 2])*(param4[num_via - 2][1] + 2 * param4[num_via - 2][2] + 3 * param4[num_via - 2][3]);

	// Calculating Acl
	JointAclArray[num_samples][0] = (1 / pow(h[num_via - 2], 2))*(2 * param1[num_via - 2][2] + 6 * param1[num_via - 2][3]);
	JointAclArray[num_samples][1] = (1 / pow(h[num_via - 2], 2))*(2 * param2[num_via - 2][2] + 6 * param2[num_via - 2][3]);
	JointAclArray[num_samples][2] = (1 / pow(h[num_via - 2], 2))*(2 * param3[num_via - 2][2] + 6 * param3[num_via - 2][3]);
	JointAclArray[num_samples][3] = (1 / pow(h[num_via - 2], 2))*(2 * param4[num_via - 2][2] + 6 * param4[num_via - 2][3]);

	// find maximum Joint Velocities
	vect JointVel_MAX = { 0, 0, 0, 0 };
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < num_samples; j++) {

			if (fabs(JointVelArray[j][i]) > JointVel_MAX[i]) {
				JointVel_MAX[i] = fabs(JointVelArray[j][i]);
			}
		}
	}

	num_samples = num_samples + 1; //TODO: CHECK THIS LATER

	//Save sampled trajectory into txt file
	std::ofstream file("plannedP.txt");
	if (file.is_open()) {
		for (int i = 0; i < num_samples; i++)
		{
			double time = i * DELTA_T1;
			file << time << " ";
			for (int j = 0; j < VECTOR_SIZE; j++) {
				file << JointPosArray[i][j] << " ";
			}
			file << std::endl;

		}
		file.close();
	}
	else {
		std::cout << "unable to open file";
	}
	std::ofstream file2("plannedV.txt");
	if (file2.is_open()) {
		for (int i = 0; i < num_samples; i++)
		{
			double time = i * DELTA_T1;
			file2 << time << " ";
			for (int j = 0; j < VECTOR_SIZE; j++) {
				file2 << JointVelArray[i][j] << " ";
			}
			file2 << std::endl;

		}
		file2.close();
	}
	else {
		std::cout << "unable to open file";
	}
	std::ofstream file3("plannedA.txt");
	if (file3.is_open()) {
		for (int i = 0; i < num_samples; i++)
		{
			double time = i * DELTA_T1;
			file3 << time << " ";
			for (int j = 0; j < VECTOR_SIZE; j++) {
				file3 << JointAclArray[i][j] << " ";
			}
			file3 << std::endl;

		}
		file3.close();
	}
	else {
		std::cout << "unable to open file";
	}

}


// Execute Trajectory
void TraExec(vect* JointPosArray, vect* JointVelArray, vect* JointAccArray, double sampling_rate, unsigned int num_of_samples)
{
	// Move Robot to First Point
	std::cout << "Moving Robot to the initial Trajectory Point" << std::endl;

#ifdef DEBUG
	DisplayConfiguration(JointPosArray[0]);
#endif

#ifndef DEBUG
	MoveToConfiguration(JointPosArray[0], true);
#endif

	// Calculate step duration
	double mili = (1.0 / sampling_rate) * S_TO_MILIS;

	for (int i = 0; i < num_of_samples; i++)
	{
		bool success = false;
		success = MoveWithConfVelAcc(JointPosArray[i], JointVelArray[i], JointAccArray[i]);
		//success = MoveToConfiguration(JointConfigArray[i], false);
		if (!success)
		{
			std::cout << "Could Not Move to Desired Position" << std::endl;
			return;
		}
		microsleep(mili);

	}
	std::cout << "Done!" << std::endl;
}


#endif // TrajectoryPlanning_h__








