#ifndef MATRIX_H
#define MATRIX_H

#include "icall.h"

#define N 3
#define Pi 3.141593

#ifdef __cplusplus
extern "C"
{
#endif

void MatrixMul(float A[N][N], float B[N][N], float C[N][N]);
void MatirxDelta(float A[N][N], float B[N][N], float delta[N]);
void EulerToMatrix(float theta_xyz[N], float T[N][N]);
void MatrixToEuler(float T[N][N], float theta_xyz[N]);
void swap(float *a, float *b);
int inv(float a[N][N], float b[N][N]);
void MatrixToProjective(float T[N][N], float theta_xyz[3]);
#ifdef __cplusplus
}
#endif

#endif