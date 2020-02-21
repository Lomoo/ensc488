#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

//const double PI = 3.1415;

class Transform {
	double pos[4];
	double rot[4][4];
public:
	Transform();
	void set_pos(double x, double y, double z, double theta) { 
		pos[0] = x;
		pos[1] = y;
		pos[2] = z;
		pos[3] = theta;
		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {
				rot[i][j] = 0;
			}
		}
	};
	void posToRot() {
		rot[0][0] = cos(pos[3]);
		rot[0][1] = -sin(pos[3]);
		rot[0][2] = 0;
		rot[0][3] = pos[0];
		rot[1][0] = sin(pos[3]);
		rot[1][1] = cos(pos[3]);
		rot[1][2] = 0;
		rot[1][3] = pos[1];
		rot[2][0] = 0;
		rot[2][1] = 0;
		rot[2][2] = 1;
		rot[2][3] = pos[2];
		rot[3][0] = 0;
		rot[3][1] = 0;
		rot[3][2] = 0;
		rot[3][3] = 1;
	};
	void set_rot(double r11, double r12, double r13, double r14,
		double r21, double r22, double r23, double r24,
		double r31, double r32, double r33, double r34,
		double r41, double r42, double r43, double r44) {
		rot[0][0] = r11;
		rot[0][1] = r12;
		rot[0][2] = r13;
		rot[0][3] = r14;
		rot[1][0] = r21;
		rot[1][1] = r22;
		rot[1][2] = r23;
		rot[1][3] = r24;
		rot[2][0] = r31;
		rot[2][1] = r32;
		rot[2][2] = r33;
		rot[2][3] = r34;
		rot[3][0] = r41;
		rot[3][1] = r42;
		rot[3][2] = r43;
		rot[3][3] = r44;
		for (int i = 0; i <= 3; i++) {
			pos[i] = 0;
		}
	};
	void rotToPos() {
		pos[0] = rot[0][3];
		pos[1] = rot[1][3];
		pos[2] = rot[2][3];
		pos[3] = acos(rot[0][0]);
	};
};

Transform::Transform() {
	double pos[4] = {0};
	double rot[4][4] = {0};
}

int main() {
	Transform T1;
	T1.set_pos( 3, 4, 1, 3.1415 /2 );
    T1.posToRot();
	Transform T2;
	T2.set_rot(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
	T2.rotToPos();

	return 0;
}