#pragma once

#ifndef matrix_h__
#define matrix_h__

//basic 4x4 matrix functions for Robotics project
//
//
//How to use functions:
//
//  UTOI: User to Interface UTOI(configuration std::vect (x,y,z,phi))=T matrix of frame
//  ITOU: Interface to User ITOU(matrix A)=configuration std::vect (x,y,z,phi)
//  DisplayMatrix(matrix A)-> output A
//  DisplayV(std::vect v)-> output v
//  Multiply(matrix A,matrix B) = A * B
//  addMatrixes(matrix A, matrix B) = A + B
//  inverseMatrix(matrix A)=A^-1
//
//
//How to initialize a 4x4 matrix A:
//
//  matrix A(4, std::vect<double>(4));
//  A={{x,x,x,x},{x,x,x,x},{x,x,x,x},{x,x,x,x}};
//
//
//How to initialize a 4 std::vect v:
//
//  std::vect<double> v(4);
//  v={x,x,x,x};


#define  VECTOR_SIZE 4

#include <iostream>
#include <cmath>


typedef double vect[VECTOR_SIZE];

typedef double matrix[VECTOR_SIZE][VECTOR_SIZE];



void MatrixInit(matrix& a)
{
	for (int i = 0; i < VECTOR_SIZE; i++)
	{
		for (int j = 0; j < VECTOR_SIZE; j++)
		{
			a[i][j] = 0.0f;
		}
	}
}



void VectorInit(vect& a)
{
	for (int j = 0; j < VECTOR_SIZE; j++)
	{
		a[j] = 0.0f;
	}
}
// copies Vector a into Vector b
void VectorCopy(vect a, vect& b)
{
	for (int i = 0; i < VECTOR_SIZE; i++)
	{
		b[i] = a[i];
	}
}

// addMatrixes two Vectors
void VectorAdd(vect a, vect b, vect& output)
{
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		output[i] = (a[i] + b[i]);
	}
}

// copies matrix a into matrix b
void MatrixCopy(matrix a, matrix& b)
{
	for (int i = 0; i < VECTOR_SIZE; i++)
	{
		for (int j = 0; j < VECTOR_SIZE; j++)
		{
			b[i][j] = a[i][j];
		}
	}
}
// Subtract two vectors
void VectorSub(const vect a, const vect b, vect& output)
{
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		output[i] = (a[i] - b[i]);
	}
}

// Multiply vect by scalar
void VectorMulS(const vect a, double constant, vect& output)
{
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		output[i] = a[i] * constant;
	}
}

void VectorMultElement(const vect a, const vect b, vect &output)
{
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		output[i] = a[i] * b[i];
	}
}

// Magnitude of the vect
double VectorMag(const vect a)
{
	double sum = 0.0f;
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		sum += pow(a[i], 2);
	}
	return sqrt(sum);
}

// The sum of the difference of each vect component
double VectorDiffSum(const vect a, const vect b)
{

	double sum = 0.0f;
	for (unsigned int i = 0; i < VECTOR_SIZE; i++)
	{
		sum += abs(a[i] - b[i]);
	}
	return sum;
}

//converts User input to internal representation of frames ((x,y,z,phi) to matrix T)
void U2I(vect config, matrix& result) {

	config[3] = DEG2RAD(config[3]);

	matrix temp = { {cos(config[3]),-sin(config[3]),0,config[0]},
			{sin(config[3]),cos(config[3]),0,config[1]},
			{0,0,-1,config[2]},
			{0,0,0,1} };

	MatrixCopy(temp, result);
}

//converts internal to user representation of frames (matrix to vect (x,y,z,phi)
void I2U(const matrix frame, vect& result) {

	vect temp = { frame[0][3], frame[1][3], frame[2][3], RAD2DEG(atan2(frame[1][0],frame[0][0])) };
	VectorCopy(temp, result);
}

//Displays matrix
void DisplayMatrix(const matrix A) {

	for (unsigned int i = 0;i < VECTOR_SIZE;i++) {
		for (int j = 0;j < VECTOR_SIZE;j++) {
			std::cout << A[i][j] << "   ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

//Displays std::vect
void DisplayV(const vect vector) {

	for (unsigned int i = 0; i < VECTOR_SIZE; i++) {
		std::cout << vector[i] << "   ";
	}
	std::cout << std::endl << std::endl;
}

//Multiplies 2 matrices
void Multiply(const matrix A, const matrix B, matrix& result) {

	for (unsigned int i = 0;i < VECTOR_SIZE;i++) {
		for (int j = 0;j < VECTOR_SIZE;j++) {
			for (int k = 0;k < VECTOR_SIZE;k++) {
				result[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}

//Adds two matrices
void addMatrixes(const matrix A, const matrix B, matrix& result) {

	for (unsigned int i = 0;i < VECTOR_SIZE;i++) {
		for (int j = 0;j < VECTOR_SIZE;j++) {
			result[i][j] = A[i][j] + B[i][j];
		}
	}
}

//gives inverseMatrix of matrix
void inverseMatrix(const matrix A, matrix& result) {


	//Rotation part of T is just transposing
	for (unsigned int i = 0; i < VECTOR_SIZE - 1;i++) {
		for (int j = 0; j < VECTOR_SIZE - 1;j++) {
			result[i][j] = A[j][i];
		}
	}

	//new origin std::vect is -R_T*v_old
	for (int k = 0; k < VECTOR_SIZE - 1; k++) {
		for (unsigned int i = 0; i < VECTOR_SIZE - 1;i++) {
			result[k][3] += (-A[i][k])*A[i][3];
		}
	}
	//last row doesn't change
	for (unsigned int i = 0; i < VECTOR_SIZE; i++) {
		result[3][i] = A[3][i];
	}
}

#endif // matrix_h__