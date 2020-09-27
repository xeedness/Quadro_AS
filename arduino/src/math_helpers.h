/*
 * math_helpers.h
 *
 * Created: 24.09.2020 16:35:41
 *  Author: xeedn
 */ 


#ifndef MATH_HELPERS_H_
#define MATH_HELPERS_H_

// Be careful what you wish for
float determinant(float mat[4][4], int dim);
int invert(float dst[4][4], float src[4][4], int dim);
void cofactor(float dst[4][4], float src[4][4], int dim);
void transpose(float dst[4][4], float src[4][4], int dim);

void multiply(float* dst, float* a, float* b, int rowsA, int colsA, int rowsB, int colsB);
void add(float* dst, float* a, float* b, int rows, int cols);
void subtract(float* dst, float* a, float* b, int rows, int cols);

#endif /* MATH_HELPERS_H_ */