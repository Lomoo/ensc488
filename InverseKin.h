#pragma once
#ifndef InverseKin_h__
#define InverseKin_h__

#include "matrix.h"
#include "globalRobotConfig.h"
#include <math.h>
#include <limits.h>

#define MAX_NUM_SOLUTIONS 2

// INVKIN - inverseMatrix Kinematics Function 
// @input  WrelB   - T matrix of the tool frame relative to the base frame
// @input  current - current joint states {theta1,theta2,d3,theta4}  
// @output near    - nearest solution wrt current position {theta1,theta2,d3,theta4}
// @output far     - farthest solution wrt current position {theta1,theta2,d3,theta4}
// @output sol     - true of solutions exist otherwise false {theta1,theta2,d3,theta4}
void INVKIN(matrix T_BW, vect current, vect& near_sol, vect& far_sol, bool& sol)
{
	sol = false;

	// Relative to the base Frame
	vect DesiredPosition = { 0, 0, 0, 0 };
	I2U(T_BW, DesiredPosition);

	double x = DesiredPosition[0];
	double y = DesiredPosition[1];
	double z = DesiredPosition[2];
	double phi = DesiredPosition[3];

	if (sqrt(pow(x, 2) + pow(y, 2)) > (L3 + L4))
	{
		sol = false;
		std::cout << "position is outside of the workspace" << std::endl;
		return;
	}
	if (sqrt(pow(x, 2) + pow(y, 2)) < (L3 - L4))
	{
		sol = false;
		std::cout << "position is outside of the workspace" << std::endl;
		return;
	}
	if ((z > 185) || (z < 85))
	{
		sol = false;
		std::cout << "position is outside of the workspace" << std::endl;
		return;
	}

	// Calculated positions
	double theta1 = 0.0f, theta2 = 0.0f, d3 = 0.0f, theta4 = 0.0f;

	// Vector containing valid solutions
	vect solutions[MAX_NUM_SOLUTIONS] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

	// inverseMatrix calculation
	for (int sign = -1; sign <= 1; sign += 2)
	{
		double costheta2 = (pow(x, 2) + pow(y, 2) - pow(L3, 2) - pow(L4, 2)) / (2 * L3*L4);
		theta2 = atan2(sign*sqrt(1 - pow(costheta2, 2)), costheta2);
		theta1 = RAD2DEG(atan2(((L3 + L4 * cos(theta2))*y - L4 * sin(theta2)*x), ((L3 + L4 * cos(theta2))*x + L4 * sin(theta2)*y)));
		theta2 = RAD2DEG(theta2);
		d3 = L1 + L2 - z - 410 - L6;

		theta4 = theta1 + theta2 - phi;

		vect solution = { theta1, theta2, d3, theta4 };

		// Make sure solution is valid
		if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
		{
			VectorCopy(solution, solutions[(sign + 1) / 2]);
			sol = true;
		}
	}

	//if (solutions.size() == 0)
	//{
	//  sol = false;
	//}
	if (sol == true)
	{
		// Find the Nearest and Farthest Solution wrt. the current joint config
		double nearest = 999999999; // Very Large Numbers for init purposes
		double farthest = -999999999; // Very Small number for init purposes 
		for (unsigned int i = 0; i < MAX_NUM_SOLUTIONS; i++)
		{
			// Invalid Solution if Vector is all 0 (d3 cannot be equal to 0)
			if (solutions[i][2] == 0)
			{
				continue;
			}

			double diffsum = VectorDiffSum(solutions[i], current);
			if (diffsum > farthest)
			{
				farthest = diffsum;
				VectorCopy(solutions[i], far_sol);
			}
			if (diffsum < nearest)
			{
				nearest = diffsum;
				VectorCopy(solutions[i], near_sol);
			}
		}
	}




	return;
}

//tool is attached to link 3 of the manipulator.This tool is described by T_WT, the
//tool frame relative to the wrist frame. Also, a user has described his work area, the
//station frame relative to the base of the robot, as T_BS .Write the subroutine

//Procedure SOLVE(VAR - trels: frame; VAR current, near, far: vec3, VAR sol : boolean);

//where "trels" is the{ T } frame specified relative to the{ S } frame.Other parameters
//are exactly as in the INVKIN subroutine.The defmitions of{ T } and{ S } should be
//globally defined variables or constants.SOLVE should use calls to TMULT, TINVERT,
//and INVKIN.

void SOLVE(matrix T_ST, vect current, vect& near_sol, vect& far_sol, bool& sol)
{
	matrix T_BS = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
	matrix T_TW = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

	matrix T_BT = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

	matrix T_BW = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };


	inverseMatrix(T_SB, T_BS); // T_BS
	inverseMatrix(T_WT, T_TW); // T_TW
	Multiply(T_BS, T_ST, T_BT); // T_BS * T_ST = T_BT
	Multiply(T_BT, T_TW, T_BW); // T_BT * T_TW = T_BW
	INVKIN(T_BW, current, near_sol, far_sol, sol);
}



#endif // InverseKin_h__