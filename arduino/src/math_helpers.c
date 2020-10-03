/*
 * math_helpers.c
 *
 * Created: 24.09.2020 16:35:54
 *  Author: xeedn
 */ 

#include <asf.h>
#include "math_helpers.h"
#include <math.h>

int invert(float dst[4][4], float src[4][4], int dim) {
	if(dim < 1 || dim > 4) {
		return 1;
	}
	
	float fac[4][4], facT[4][4];
	float d = determinant(src, dim);
	if(d == 0) {
		//printf("Determinant is zero.\n");
		return 1;
	}
	cofactor(fac, src, dim);
	transpose(facT, fac, dim);
	
	for (int i = 0;i < dim; i++)
	{
		for (int j = 0;j < dim; j++)
		{
			dst[i][j] = facT[i][j] / d;
		}
	}	
	return 0;
}

float determinant(float a[4][4], int k)
{
	float s = 1, det = 0, b[4][4];
	int i, j, m, n, c;
	if (k == 1)
	{
		return (a[0][0]);
	}
	else
	{
		det = 0;
		for (c = 0; c < k; c++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < k; i++)
			{
				for (j = 0 ;j < k; j++)
				{
					b[i][j] = 0;
					if (i != 0 && j != c)
					{
						b[m][n] = a[i][j];
						if (n < (k - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (a[0][c] * determinant(b, k - 1));
			s = -1 * s;
		}
	}
	
	return (det);
}

void cofactor(float dst[4][4], float src[4][4], int f)
{
	float b[4][4];
	int p, q, m, n, i, j;
	for (q = 0;q < f; q++)
	{
		for (p = 0;p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < f; i++)
			{
				for (j = 0;j < f; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = src[i][j];
						if (n < (f - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			dst[q][p] = pow(-1, q + p) * determinant(b, f - 1);
		}
	}
}

void transpose(float dst[4][4], float src[4][4], int dim)
{
	int i, j;	
	for (i = 0;i < dim; i++)
	{
		for (j = 0;j < dim; j++)
		{
			dst[i][j] = src[j][i];
		}
	}
}

void multiply(float* dst, float* a, float* b, int rowsA, int colsA, int rowsB, int colsB) {
	for (int i = 0; i < rowsA; i++) {
		for (int j = 0; j < colsB; j++) {
			dst[i*colsB+j] = 0;
		}
	}

	for (int i = 0; i < rowsA; i++) {
		for (int j = 0; j < colsB; j++) {
			for (int k = 0; k < colsA; k++) {
				dst[i*colsB+j] += a[i*colsA+k] * b[k*colsB+j];
			}
		}
	}
}

void add(float* dst, float* a, float* b, int rows, int cols) {
	for(int i = 0; i < rows; i++) {
		for(int j = 0; j < cols; j++) {
			dst[i*cols+j] = a[i*cols+j] + b[i*cols+j];
		}
	}
}

void subtract(float* dst, float* a, float* b, int rows, int cols) {
	for(int i = 0; i < rows; i++) {
		for(int j = 0; j < cols; j++) {
			dst[i*cols+j] = a[i*cols+j] - b[i*cols+j];
		}
	}
}