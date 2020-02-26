//
//  WHERE.h
//  WHERE
//
//

#ifndef WHERE_h
#define WHERE_h

#include "matrix.h"
#include "globalRobotConfig.h"
#include <stdio.h>



// Outputs the Matrix relative to the base
void kinematics(double theta1, double theta2, double d3, double theta4, matrix& output)
{
	theta1 = DEG2RAD(theta1);
	theta2 = DEG2RAD(theta2);
	theta4 = DEG2RAD(theta4);

	//assigning T matrices

	matrix T01 = { {cos(theta1), -sin(theta1), 0, 0}, {sin(theta1), cos(theta1), 0, 0}, {0, 0, 1, L1}, {0, 0, 0, 1} };
	matrix T12 = { {cos(theta2), -sin(theta2), 0, L3}, {sin(theta2), cos(theta2), 0, 0}, {0, 0, 1, L2}, {0, 0, 0, 1} };
	matrix T23 = { {1, 0, 0, L4}, {0, -1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1} };
	matrix T34 = { {cos(theta4), -sin(theta4), 0, 0}, {sin(theta4), cos(theta4), 0, 0}, {0, 0, 1, L6 + L5 + d3}, {0, 0, 0, 1} };

	//multiplying them to achieve T05

	matrix T02 = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
	Multiply(T01, T12, T02);
	matrix T03 = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
	Multiply(T02, T23, T03);
	matrix T04 = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
	Multiply(T03, T34, T04);

	MatrixCopy(T04, output);
	//DisplayMatrix(T04);

}
// Outputs Vector relative to the station frame
void WHERE(double theta1, double theta2, double d3, double theta4, vect& output)
{
	vect tempVect = { 0, 0, 0, 0 };
	matrix outputMat = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

	//double check if within limits
	if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
	{
		//calling function KIN to calculate T05
		matrix T04 = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
		kinematics(theta1, theta2, d3, theta4, T04);
		matrix WRelB = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
		Multiply(T_SB, T04, WRelB);
		Multiply(WRelB, T_WT, outputMat);

		//output = Multiply(Multiply(BRelS, KIN(theta1, theta2, d3, theta4)), TRelW);
	}
	else
	{
		VectorCopy(tempVect, output);
		std::cout << "Input parameters not within limits. \n";
	}

	//converting matrix into vector for user
	I2U(outputMat, tempVect);
	VectorCopy(tempVect, output);
}

#endif /* WHERE_h */