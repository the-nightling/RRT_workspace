#ifndef RRT_H
#define RRT_H

/////////////////////////////////////////////////////////////////
// MACROS
/////////////////////////////////////////////////////////////////

// defining macros for phase space limits
#define ANG_POS_MIN -M_PI		// minimum angular position
#define ANG_POS_MAX M_PI		// maximum angular position
#define ANG_VEL_MIN -8.0		// minimum angular velocity
#define ANG_VEL_MAX 8.0         // maximum angular velocity

// RRT algorithm macros
#define NUM_OF_ITERATIONS 172800
#define LENGTH_OF_SOLN_PATH 30
#define NUM_OF_GOAL_STATES 8

/////////////////////////////////////////////////////////////////
// FUNCTION HEADERS
/////////////////////////////////////////////////////////////////

double generateRandomDouble(double min, double max);
void euclidianDistSquare(double * A, double B[][2], int lengthOfB, double* listOfDistSq);
int findMin(double array[], int lengthOfArray);
void pendulumDynamics(double* x, double u, double* next_state);
int RRT_kernel(double control_solution[]);


#endif // RRT_H
