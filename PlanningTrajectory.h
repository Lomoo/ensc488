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

	//for (int i = 0; i < num_via; i++) {
	//	v1_via[i] = 0;
	//	v2_via[i] = 0;
	//	v3_via[i] = 0;
	//	v4_via[i] = 0;

	//}

	////calculate via point velocities as average//
	////inital and final velocity remain 0 to obtain a smooth start/stop

	//for (int i = 1; i < num_via-1; i++) {
	//	v1_via[i] = (((theta1_via[i] - theta1_via[i - 1]) / h[i - 1]) + ((theta1_via[i + 1] - theta1_via[i]) / h[i])) / 2;
	//	v2_via[i] = (((theta2_via[i] - theta2_via[i - 1]) / h[i - 1]) + ((theta2_via[i + 1] - theta2_via[i]) / h[i])) / 2;
	//	v3_via[i] = (((d3_via[i] - d3_via[i - 1]) / h[i - 1]) + ((d3_via[i + 1] - d3_via[i]) / h[i])) / 2;
	//	v4_via[i] = (((theta4_via[i] - theta4_via[i - 1]) / h[i - 1]) + ((theta4_via[i + 1] - theta4_via[i]) / h[i])) / 2;
	//}

	double a_theta1[MAX_VIA_POINTS];
	double b_theta1[MAX_VIA_POINTS];
	double c_theta1[MAX_VIA_POINTS];
	double d_theta1[MAX_VIA_POINTS];
	double a_theta2[MAX_VIA_POINTS];
	double b_theta2[MAX_VIA_POINTS];
	double c_theta2[MAX_VIA_POINTS];
	double d_theta2[MAX_VIA_POINTS];
	double a_d3[MAX_VIA_POINTS];
	double b_d3[MAX_VIA_POINTS];
	double c_d3[MAX_VIA_POINTS];
	double d_d3[MAX_VIA_POINTS];
	double a_theta4[MAX_VIA_POINTS];
	double b_theta4[MAX_VIA_POINTS];
	double c_theta4[MAX_VIA_POINTS];
	double d_theta4[MAX_VIA_POINTS];
	double w_theta1 = 0;
	double w_theta2 = 0;
	double w_theta3 = 0;
	double w_theta4 = 0;


	a_theta1[0] = 0;
	for (int i = 0; i < num_via - 2; i++) {
		b_theta1[i] = 4 / h[i] + 4 / h[i + 1];
		b_theta2[i] = 4 / h[i] + 4 / h[i + 1];
		b_d3[i] = 4 / h[i] + 4 / h[i + 1];
		b_theta4[i] = 4 / h[i] + 4 / h[i + 1];

		if (i != num_via - 2) {
			c_theta1[i] = 2 / h[i + 1];
			c_theta2[i] = 2 / h[i + 1];
			c_d3[i] = 2 / h[i + 1];
			c_theta4[i] = 2 / h[i + 1];
		}
		if (i != 0) {
			a_theta1[i] = 2 / h[i];
			a_theta2[i] = 2 / h[i];
			a_d3[i] = 2 / h[i];
			a_theta4[i] = 2 / h[i];
		}
		d_theta1[i] = 6.*((theta1_via[i + 1] - theta1_via[i]) / pow(h[i], 2) + (theta1_via[i + 2] - theta1_via[i + 1]) / pow(h[i + 1], 2));
		d_theta2[i] = 6.*((theta2_via[i + 1] - theta2_via[i]) / pow(h[i], 2) + (theta2_via[i + 2] - theta2_via[i + 1]) / pow(h[i + 1], 2));
		d_d3[i] = 6.*((d3_via[i + 1] - d3_via[i]) / pow(h[i], 2) + (d3_via[i + 2] - d3_via[i + 1]) / pow(h[i + 1], 2));
		d_theta4[i] = 6.*((theta4_via[i + 1] - theta4_via[i]) / pow(h[i], 2) + (theta4_via[i + 2] - theta4_via[i + 1]) / pow(h[i + 1], 2));
	}

	for (int i = 1; i < num_via - 2; i++) {
		w_theta1 = a_theta1[i] / b_theta1[i - 1];
		b_theta1[i] = b_theta1[i] - w_theta1 * c_theta1[i - 1];
		d_theta1[i] = d_theta1[i] - w_theta1 * d_theta1[i - 1];

		w_theta2 = a_theta2[i] / b_theta2[i - 1];
		b_theta2[i] = b_theta2[i] - w_theta2 * c_theta2[i - 1];
		d_theta2[i] = d_theta2[i] - w_theta2 * d_theta2[i - 1];

		w_theta3 = a_d3[i] / b_d3[i - 1];
		b_d3[i] = b_d3[i] - w_theta3 * c_d3[i - 1];
		d_d3[i] = d_d3[i] - w_theta3 * d_d3[i - 1];

		w_theta4 = a_theta4[i] / b_theta4[i - 1];
		b_theta4[i] = b_theta4[i] - w_theta4 * c_theta4[i - 1];
		d_theta4[i] = d_theta4[i] - w_theta4 * d_theta4[i - 1];

	}

	v1_via[num_via - 2] = d_theta1[num_via - 3] / b_theta1[num_via - 3];
	v2_via[num_via - 2] = d_theta2[num_via - 3] / b_theta2[num_via - 3];
	v3_via[num_via - 2] = d_d3[num_via - 3] / b_d3[num_via - 3];
	v4_via[num_via - 2] = d_theta4[num_via - 3] / b_theta4[num_via - 3];

	for (int i = num_via - 1; i >= 0; i--) {
		if (i == 0 || i == num_via - 1) {
			v1_via[i] = 0;
			v2_via[i] = 0;
			v3_via[i] = 0;
			v4_via[i] = 0;
			continue;
		}
		else if (i == num_via - 2) {
			continue;
		}
		v1_via[i] = (d_theta1[i - 1] - v1_via[i + 1] * c_theta1[i - 1]) / b_theta1[i - 1];
		v2_via[i] = (d_theta2[i - 1] - v2_via[i + 1] * c_theta2[i - 1]) / b_theta2[i - 1];
		v3_via[i] = (d_d3[i - 1] - v3_via[i + 1] * c_d3[i - 1]) / b_d3[i - 1];
		v4_via[i] = (d_theta4[i - 1] - v4_via[i + 1] * c_theta4[i - 1]) / b_theta4[i - 1];

	}


	//calculate parameters//

	// for theta1:
	for (int i = 0; i < num_via - 1; i++) {
		param1[i][0] = theta1_via[i];  //parameter a
		param1[i][1] = v1_via[i]; //parameter b
		param1[i][2] = 3 * (theta1_via[i + 1] - theta1_via[i]) / pow(h[i], 2) - 2 * v1_via[i] / h[i] - v1_via[i + 1] / h[i];//parameter d
		param1[i][3] = -2 * (theta1_via[i + 1] - theta1_via[i]) / pow(h[i], 3) + (v1_via[i + 1] + v1_via[i]) / pow(h[i], 2); //parameter c
	}

	//for theta2:
	for (int i = 0; i < num_via - 1; i++) {
		param2[i][0] = theta2_via[i];  //parameter a
		param2[i][1] = v2_via[i]; //parameter b
		param2[i][2] = 3 * (theta2_via[i + 1] - theta2_via[i]) / pow(h[i], 2) - 2 * v2_via[i] / h[i] - v2_via[i + 1] / h[i];//parameter d
		param2[i][3] = -2 * (theta2_via[i + 1] - theta2_via[i]) / pow(h[i], 3) + (v2_via[i + 1] + v2_via[i]) / pow(h[i], 2); //parameter c
	}

	//for theta3:
	for (int i = 0; i < num_via - 1; i++) {
		param3[i][0] = d3_via[i];  //parameter a
		param3[i][1] = v3_via[i]; //parameter b
		param3[i][2] = 3 * (d3_via[i + 1] - d3_via[i]) / pow(h[i], 2) - 2 * v3_via[i] / h[i] - v3_via[i + 1] / h[i];//parameter d
		param3[i][3] = -2 * (d3_via[i + 1] - d3_via[i]) / pow(h[i], 3) + (v3_via[i + 1] + v3_via[i]) / pow(h[i], 2); //parameter c
	}

	//for theta4:
	for (int i = 0; i < num_via - 1; i++) {
		param4[i][0] = theta4_via[i];  //parameter a
		param4[i][1] = v4_via[i]; //parameter b
		param4[i][2] = 3 * (theta4_via[i + 1] - theta4_via[i]) / pow(h[i], 2) - 2 * v4_via[i] / h[i] - v4_via[i + 1] / h[i];//parameter d
		param4[i][3] = -2 * (theta4_via[i + 1] - theta4_via[i]) / pow(h[i], 3) + (v4_via[i + 1] + v4_via[i]) / pow(h[i], 2); //parameter c
	}
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

	double tau = 0;
	//num_seg_samples[i] = (via_times[i + 1] - via_times[i]) * SAMPLING_RATE_T1;
	//seg_offset[i + 1] = seg_offset[i] + num_seg_samples[i];

	// Loop through segments
	for (int cur_seg = 0; cur_seg < num_via - 1; cur_seg++)
	{
		// Parameterized Time
		double time = h[cur_seg] / (num_samples / (num_via - 1));

		// Sample within segments
		for (int cur_sample = (num_samples / (num_via - 1))* cur_seg; cur_sample < (num_samples / (num_via - 1))* cur_seg + (num_samples / (num_via - 1)); cur_sample++)
		{



			// Calculating Pos

			JointPosArray[cur_sample][0] = param1[cur_seg][0] + param1[cur_seg][1] * (tau - via_times[cur_seg]) + param1[cur_seg][2] * pow((tau - via_times[cur_seg]), 2) + param1[cur_seg][3] * pow((tau - via_times[cur_seg]), 3);
			JointPosArray[cur_sample][1] = param2[cur_seg][0] + param2[cur_seg][1] * (tau - via_times[cur_seg]) + param2[cur_seg][2] * pow((tau - via_times[cur_seg]), 2) + param2[cur_seg][3] * pow((tau - via_times[cur_seg]), 3);
			JointPosArray[cur_sample][2] = param3[cur_seg][0] + param3[cur_seg][1] * (tau - via_times[cur_seg]) + param3[cur_seg][2] * pow((tau - via_times[cur_seg]), 2) + param3[cur_seg][3] * pow((tau - via_times[cur_seg]), 3);
			JointPosArray[cur_sample][3] = param4[cur_seg][0] + param4[cur_seg][1] * (tau - via_times[cur_seg]) + param4[cur_seg][2] * pow((tau - via_times[cur_seg]), 2) + param4[cur_seg][3] * pow((tau - via_times[cur_seg]), 3);

			// Calculating Vel
			JointVelArray[cur_sample][0] = param1[cur_seg][1] + 2 * param1[cur_seg][2] * (tau - via_times[cur_seg]) + 3 * param1[cur_seg][3] * pow((tau - via_times[cur_seg]), 2);
			JointVelArray[cur_sample][1] = param2[cur_seg][1] + 2 * param2[cur_seg][2] * (tau - via_times[cur_seg]) + 3 * param2[cur_seg][3] * pow((tau - via_times[cur_seg]), 2);
			JointVelArray[cur_sample][2] = param3[cur_seg][1] + 2 * param3[cur_seg][2] * (tau - via_times[cur_seg]) + 3 * param3[cur_seg][3] * pow((tau - via_times[cur_seg]), 2);
			JointVelArray[cur_sample][3] = param4[cur_seg][1] + 2 * param4[cur_seg][2] * (tau - via_times[cur_seg]) + 3 * param4[cur_seg][3] * pow((tau - via_times[cur_seg]), 2);

			// Calculating Acc
			JointAclArray[cur_sample][0] = 2 * param1[cur_seg][2] + 6 * param1[cur_seg][3] * (tau - via_times[cur_seg]);
			JointAclArray[cur_sample][1] = 2 * param2[cur_seg][2] + 6 * param2[cur_seg][3] * (tau - via_times[cur_seg]);
			JointAclArray[cur_sample][2] = 2 * param3[cur_seg][2] + 6 * param3[cur_seg][3] * (tau - via_times[cur_seg]);
			JointAclArray[cur_sample][3] = 2 * param4[cur_seg][2] + 6 * param4[cur_seg][3] * (tau - via_times[cur_seg]);
			tau += time;

			


			
		}
	}

	//Calculating last sample
	//Calculating Pos
	JointPosArray[num_samples][0] = param1[num_via - 2][0] + param1[num_via - 2][1] * (tau - via_times[num_via - 2]) + param1[num_via - 2][2] * pow((tau - via_times[num_via - 2]), 2) + param1[num_via - 2][3] * pow((tau - via_times[num_via - 2]), 3);
	JointPosArray[num_samples][1] = param2[num_via - 2][0] + param2[num_via - 2][1] * (tau - via_times[num_via - 2]) + param2[num_via - 2][2] * pow((tau - via_times[num_via - 2]), 2) + param2[num_via - 2][3] * pow((tau - via_times[num_via - 2]), 3);
	JointPosArray[num_samples][2] = param3[num_via - 2][0] + param3[num_via - 2][1] * (tau - via_times[num_via - 2]) + param3[num_via - 2][2] * pow((tau - via_times[num_via - 2]), 2) + param3[num_via - 2][3] * pow((tau - via_times[num_via - 2]), 3);
	JointPosArray[num_samples][3] = param4[num_via - 2][0] + param4[num_via - 2][1] * (tau - via_times[num_via - 2]) + param4[num_via - 2][2] * pow((tau - via_times[num_via - 2]), 2) + param4[num_via - 2][3] * pow((tau - via_times[num_via - 2]), 3);
	// Calculating Vel
	JointVelArray[num_samples][0] = 0;
	JointVelArray[num_samples][1] = 0;
	JointVelArray[num_samples][2] = 0;
	JointVelArray[num_samples][3] = 0;

	JointAclArray[num_samples][0] = 2 * param1[num_via - 2][2] + 6 * param1[num_via - 2][3] * (tau - via_times[num_via - 2]);
	JointAclArray[num_samples][1] = 2 * param2[num_via - 2][2] + 6 * param2[num_via - 2][3] * (tau - via_times[num_via - 2]);
	JointAclArray[num_samples][2] = 2 * param3[num_via - 2][2] + 6 * param3[num_via - 2][3] * (tau - via_times[num_via - 2]);
	JointAclArray[num_samples][3] = 2 * param4[num_via - 2][2] + 6 * param4[num_via - 2][3] * (tau - via_times[num_via - 2]);

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
	std::ofstream file5("plannedCart.txt");
	vect CurPositionVect = { 0, 0, 0, 0 };
	if (file5.is_open()) {
		for (int i = 0; i < num_samples; i++)
		{
			double time = i * DELTA_T1;
			file5 << time << " ";
			WHERE(JointPosArray[i][0], JointPosArray[i][1], JointPosArray[i][2], JointPosArray[i][3], CurPositionVect);
			for (int j = 0; j < VECTOR_SIZE; j++) {
				file5 << CurPositionVect[j] << " ";
			}
			file5 << std::endl;

		}
		file5.close();
	}
	else {
		std::cout << "unable to open file";
	}

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

bool TraOutOfLimits(vect* JointPosArray, vect* JointVelArray, vect* JointAccArray, unsigned int num_of_samples) {
	
	for (int i = 0; i < num_of_samples; i++) 
	{
		
		if (VelTheta1Check(JointVelArray[i][0]) || VelTheta2Check(JointVelArray[i][1]) || VelD3Check(JointVelArray[i][2]) || VelTheta4Check(JointVelArray[i][3]) ||
			AccTheta1Check(JointVelArray[i][0]) || AccTheta2Check(JointVelArray[i][1]) || AccD3Check(JointVelArray[i][2]) || AccTheta4Check(JointVelArray[i][3])) 
		{
		
			if (VelTheta1Check(JointVelArray[i][0]))
			{
				std::cout << "Joint 1 Velocity of " << JointVelArray[i][0] << " m/s reached, exiting" << std::endl;
			}
			else if (VelTheta2Check(JointVelArray[i][1]))
			{
				std::cout << "Joint 2 Velocity of " << JointVelArray[i][1] << " m/s reached, exiting" << std::endl;
			}
			else if (VelD3Check(JointVelArray[i][2]))
			{
				std::cout << "Joint 3 Velocity of " << JointVelArray[i][2] << " m/s reached, exiting" << std::endl;
			}
			else if (VelTheta4Check(JointVelArray[i][3]))
			{
				std::cout << "Joint 4 Velocity of " << JointVelArray[i][3] << " m/s reached, exiting" << std::endl;
			}


			if (AccTheta1Check(JointVelArray[i][0]))
			{
				std::cout << "Joint 1 Acceleration of " << JointAccArray[i][0] << " m/s� reached, exiting" << std::endl;
			}
			else if (AccTheta2Check(JointVelArray[i][1]))
			{
				std::cout << "Joint 2 Acceleration of " << JointAccArray[i][1] << " m/s� reached, exiting" << std::endl;
			}
			else if (AccD3Check(JointVelArray[i][2]))
			{
				std::cout << "Joint 3 Acceleration of " << JointAccArray[i][2] << " m/s� reached, exiting" << std::endl;
			}
			else if (AccTheta4Check(JointVelArray[i][3]))
			{
				std::cout << "Joint 4 Acceleration of " << JointAccArray[i][3] << " m/s� reached, exiting" << std::endl;
			}
			
			
			return true;
		}
	}
	return false;

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







