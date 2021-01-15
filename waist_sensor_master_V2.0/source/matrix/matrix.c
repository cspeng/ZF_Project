
#include "stdio.h"
#include "math.h"
#include "matrix.h"



void MatirxDelta(float a[N][N], float b[N][N], float delta[N])
{
	float ab, a1, b1;
	int i;
	for (i = 0;i < 3;i++)
	{
		ab = a[0][i] * b[0][i] + a[1][i] * b[1][i]+ a[2][i] * b[2][i];
		a1 = sqrt(a[0][i] * a[0][i] + a[1][i] * a[1][i] + a[2][i] * a[2][i]);
		b1 = sqrt(b[0][i] * b[0][i] + b[1][i] * b[1][i] + b[2][i] * b[2][i]);
		delta[i] = acos(ab / a1 / b1)*180/Pi;
	}
}
void MatrixMul(float A[N][N], float B[N][N], float C[N][N])
{
	int m = N;
	int p = N;
	int n = N;
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < p; j++)
		{
			C[i][j] = 0;
			for (int k = 0; k < n; k++)
			{
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}

int inv(float a[N][N], float b[N][N])
{
	float *is, *js;
	int i, j, k;
	int n = N;
	/*
	for (i = 0;i<n;i++)
	{
		putchar('\n');
		for (j = 0;j<n;j++)
			printf("%f  ", *(p + i*n + j));
	}
	puts("nnnn");
	*/
	for (int i = 0;i < N;i++) 
		for (int j = 0;j<N;j++) 
			b[i][j] = a[i][j];
    
	float temp, fmax;
	float *p = b[0];
    
	is = (float *) ICall_malloc(n * sizeof(float));
	js = (float *) ICall_malloc(n * sizeof(float));
    
	for (k = 0;k<n;k++)
	{
		fmax = 0.0;
		for (i = k;i<n;i++)
			for (j = k;j<n;j++)
			{
				temp = fabs(*(p + i*n + j));//找最大值
				if (temp>fmax)
				{
					fmax = temp;
					is[k] = i;js[k] = j;
				}
			}
		if ((fmax + 1.0) == 1.0)
		{
			ICall_free(is);ICall_free(js);
			//SDITask_PrintfToUART("no inv");
			return(0);
		}
        
		if ((i = is[k]) != k)
			for (j = 0;j<n;j++)
				swap(p+k*n + j, p+i*n + j);//交换指针
        
		if ((j = js[k]) != k)
			for (i = 0;i<n;i++)
				swap(p+i*n + k, p+i*n + j);  //交换指针
        
		p[k*n + k] = 1.0 / p[k*n + k];
        
		for (j = 0;j<n;j++)
			if (j != k)
				p[k*n + j] *= p[k*n + k];
        
		for (i = 0;i<n;i++)
			if (i != k)
				for (j = 0;j<n;j++)
					if (j != k)
						p[i*n + j] = p[i*n + j] - p[i*n + k] * p[k*n + j];
        
		for (i = 0;i<n;i++)
			if (i != k)
				p[i*n + k] *= -p[k*n + k];
	}
    
	for (k = n - 1;k >= 0;k--)
	{
		if ((j = js[k]) != k)
			for (i = 0;i<n;i++)
				swap((p + j*n + i), (p + k*n + i));
		if ((i = is[k]) != k)
			for (j = 0;j<n;j++)
				swap(p + j*n + i, p + j*n + k);
	}
    
	ICall_free(is);
	ICall_free(js);
	return 1;
}


void swap(float *a, float *b)
{
	float c;
	c = *a;
	*a = *b;
	*b = c;
}


void EulerToMatrix(float theta_xyz[3], float T[N][N])
{
	float theta_z, theta_y, theta_x, Q[4];
	float cos_z_2, cos_y_2, cos_x_2, sin_z_2, sin_y_2, sin_x_2;
	theta_z = theta_xyz[2] * Pi / 180;
	theta_y = theta_xyz[1] * Pi / 180;
	theta_x = theta_xyz[0] * Pi / 180;

	cos_z_2 = cos(0.5*theta_z);
	cos_y_2 = cos(0.5*theta_y);
	cos_x_2 = cos(0.5*theta_x);

	sin_z_2 = sin(0.5*theta_z);
	sin_y_2 = sin(0.5*theta_y);
	sin_x_2 = sin(0.5*theta_x);

	Q[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
	Q[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
	Q[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
	Q[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;

	T[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
	T[0][1] = 2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	T[0][2] = 2 * (Q[1] * Q[3] + Q[0] * Q[2]);

	T[1][0] = 2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	T[1][1] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
	T[1][2] = 2 * (Q[2] * Q[3] - Q[0] * Q[1]);

	T[2][0] = 2 * (Q[1] * Q[3] - Q[0] * Q[2]);
	T[2][1] = 2 * (Q[2] * Q[3] + Q[0] * Q[1]);
	T[2][2] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
}

void MatrixToEuler(float T[N][N], float theta_xyz[3])
{
	float Pitch;
	float Roll;
	float Yaw;

	Pitch = asin(-T[2][0]);
	Roll = atan(T[2][1] / T[2][2]);
	Yaw = atan(T[1][0] / T[0][0]);

	if (T[2][2]<0)
	{
		if (Roll < 0)
		{
			Roll = Roll + Pi;
		}
		else
		{
			Roll = Roll - Pi;
		}
	}

	if (T[0][0]<0)
	{
		if (T[1][0]>0)
		{
			Yaw = Yaw + Pi;
		}
		else
		{
			Yaw = Yaw - Pi;
		}
	}
	theta_xyz[0] = Roll * 180 / Pi;
	theta_xyz[1] = Pitch * 180 / Pi;
	theta_xyz[2] = Yaw * 180 / Pi;
}


void MatrixToProjective(float T[N][N], float theta_xyz[3])
{
	//output [-180,180]
/*

	for (int i = 0;i < 3;i++)
	{
		if (T[0][i] == 0)
		{
			if (T[1][i] > 0) 
				theta_xyz[i] =90;
			else if (T[1][i] < 0)
				theta_xyz[i] = 270;
			else
				theta_xyz[i] = 0;
		}
		else if (T[0][i]>0)
		{
			theta_xyz[i]= atan(T[1][i]/ T[0][i]) * 180 / Pi;
		}
		else if (T[0][i]<0)
		{
			theta_xyz[i] = atan2(T[1][i]/ T[0][i]) * 180 / Pi+180;
		}
	}
*/
	for (int i = 0;i < 3;i++)
			theta_xyz[i] = atan2(T[1][i] , T[0][i]) * 180 / Pi;

	//y轴夹角要从投影的y轴相比
	theta_xyz[1] = theta_xyz[1] - 90;
	for (int i = 0;i < 3;i++)
	{
		if (theta_xyz[i] < -180) theta_xyz[i] = theta_xyz[i] + 360;
		if (theta_xyz[i] > 180) theta_xyz[i] = theta_xyz[i] - 360;
		theta_xyz[i] = -theta_xyz[i];// 求反，两脚张开，角度是正数
	}

}



