#pragma once

typedef struct {
	double x;
	double y;
	double z;
} XYZ;

typedef struct {
	double min;
	double max;
} MinMax;


#pragma region micros

#define MathPI 3.14159265358979323846
#define MathSin(x) sin(x)
#define MathCos(x) cos(x)
#define MathAtan2(y,x) atan2(y,x)
#define MathSqrt(x) sqrt(x)
#define MathAbs(x) abs(x)
#define MathCeiling(x) ceil(x)
#define MathAcos(x) acos(x)
#define MathTan(x) tan(x)


#pragma endregion micros