////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "RRT.cuh"


////////////////////////////////////////////
// CUDA KERNELS
////////////////////////////////////////////

/*
 * Initializes CUDA RNG
 */
__global__ void RNG_setup_kernel(curandState *state) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;		// thread id
	curand_init(1234, idx, 0, &state[idx]);	// using seed 1234 (change to time at a later stage)
}

/*
 * Initializes adjacent matrix
 */
__global__ void init_adj_matrix_kernel(int * adjacency_matrix){
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	for(int i=0; i < NUM_THREADS*NUM_BLOCKS; i++){
		int index = idx * NUM_THREADS*NUM_BLOCKS + i;
		if(index % (NUM_THREADS*NUM_BLOCKS + 1) == 0){
			adjacency_matrix[index] = 0;
		}else{
			adjacency_matrix[index] = 9999;
			//adjacency_matrix[index] = 0;
		}
	}
}

/*
 * Main kernel; Contains RRT algorithm
 */
__global__ void RRT_kernel(curandState *my_curandstate, int *adjacency_matrix,
		double * path_solutions, double * control_solutions, double* tmp) {

	int idx = blockIdx.x * blockDim.x + threadIdx.x;		// thread id

	// computing initial state
	double start_state[] = { ANG_POS_MIN, ANG_VEL_MIN}; // initial state; angle position measured from x-axis
	start_state[0] += ((idx % GRID_X) * 2 * DELTA_X) + (2 * DELTA_X);
	start_state[1] += (((idx / GRID_X) % (GRID_Y*NUM_BLOCKS)) * 2 * DELTA_Y) + (2 * DELTA_Y);

	tmp[2*idx] = start_state[0];
	tmp[2*idx+1] = start_state[1];

	// automate goal placement around initial state
	double end_state[NUM_OF_GOAL_STATES][DIMENSIONS] = {{0}};
	int goal_idx;
	for(goal_idx = 0; goal_idx < pow((float)3,(float)DIMENSIONS); goal_idx++)
	{
		if(goal_idx < NUM_OF_GOAL_STATES/2){
			end_state[goal_idx][0] = start_state[0] + ((goal_idx%3) - 1)*2*DELTA_X;
			end_state[goal_idx][1] = start_state[1] + (((goal_idx/3)%3) - 1)*2*DELTA_Y;
		}else if(goal_idx > NUM_OF_GOAL_STATES/2){
			end_state[goal_idx-1][0] = start_state[0] + ((goal_idx%3) - 1)*2*DELTA_X;
			end_state[goal_idx-1][1] = start_state[1] + (((goal_idx/3)%3) - 1)*2*DELTA_Y;
		}

	}

	double state_limits[2][2] = {
			{ start_state[0] - 3 * DELTA_X, start_state[0] + 3 * DELTA_X },
			{ start_state[1] - 3 * DELTA_Y, start_state[1] + 3 * DELTA_Y }
	}; // state limits; angular position between -pi & pi rad; angular velocity between -10 & 10 rad/s

	// control torques to be used: linspace(-5,5,20)
	//*
	double discrete_control_torques[] = { -5.0000, -4.4737, -3.9474, -3.4211, -2.8947, -2.3684,
			-1.8421, -1.3158, -0.7895, -0.2632, 5.0000, 4.4737, 3.9474, 3.4211,
			2.8947, 2.3684, 1.8421, 1.3158, 0.7895, 0.2632 };
	//*/
	/*
	double discrete_control_torques[] = { -1.0000, -0.8947, -0.7895, -0.6842, -0.5789, -0.4737, -0.3684, -0.2632, -0.1579, -0.0526,
			1.0000, 0.8947, 0.7895, 0.6842, 0.5789, 0.4737, 0.3684, 0.2632, 0.1579, 0.0526};
	//*/
	int number_of_discrete_torques = (int) (sizeof(discrete_control_torques) / sizeof(discrete_control_torques[0]));

	double time_step = 0.02; // time interval between application of subsequent control torques

	// static memory allocation
	double random_state[DIMENSIONS];        // stores a state
	double next_state[DIMENSIONS];

	double RRT_tree[NUM_OF_ITERATIONS][DIMENSIONS];	// stores tree
	int x, y;
	for (x = 0; x < NUM_OF_ITERATIONS; x++) {	// initialize tree to initial state
		RRT_tree[x][0] = start_state[0];
		RRT_tree[x][1] = start_state[1];
	}

	//int adjMatrix[NUM_THREADS][NUM_THREADS];
	//memset(adjMatrix, 0, sizeof(int)*NUM_THREADS*NUM_THREADS);

	int parent_state_index[NUM_OF_ITERATIONS]; // stores index of parent state for each state in graph RRT_tree
	int control_action_index[NUM_OF_ITERATIONS]; // stores index of control actions in discrete_control_torques (each state will use a control action value in discrete_control_torques)
	double u_path[NUM_OF_GOAL_STATES][LENGTH_OF_SOLN_PATH]; // stores sequence of control actions (solution to problem)
	double x_path[NUM_OF_GOAL_STATES][LENGTH_OF_SOLN_PATH][DIMENSIONS];
	for (y = 0; y < NUM_OF_GOAL_STATES; y++) {
		for (x = 0; x < LENGTH_OF_SOLN_PATH; x++) {	// initialize tree to initial state
			x_path[y][x][0] = 0;
			x_path[y][x][1] = 0;
			u_path[y][x] = 0;
		}
	}
	int state_index = 0;    // stores sequence of states joining initial to goal state
	double temp_achievable_states[20][DIMENSIONS]; // stores temporary achievable states from a particular vertex; 20 is length of discrete_control_torques

	double distance_square_values[NUM_OF_ITERATIONS];  // stores distance square values

	int goal_index;
	int not_found[NUM_OF_GOAL_STATES] = {0};
	for(int i=0; i < NUM_OF_GOAL_STATES;i++)
		not_found[i] = 1;
	int weight = 0;

	// keep growing RRT until goal found or run out of iterations
	int iteration;
	for (iteration = 1; iteration < NUM_OF_ITERATIONS; iteration++) {
		// get random state
		random_state[0] = curand_uniform(my_curandstate + idx) * (state_limits[0][1] - state_limits[0][0]) + state_limits[0][0];
		random_state[1] = curand_uniform(my_curandstate + idx) * (state_limits[1][1] - state_limits[1][0]) + state_limits[1][0];

		// find distances between that state point and every vertex in RRT
		euclidianDistSquare(random_state, RRT_tree, iteration, distance_square_values);

		// select RRT vertex closest to the state point
		int nearest_state_index = findMin(distance_square_values, iteration);

		// from the closest RRT vertex, compute all the states that can be reached,
		// given the pendulum dynamics and available torques
		int ui;
		for (ui = 0; ui < number_of_discrete_torques; ui++) {
			pendulumDynamics(RRT_tree[nearest_state_index], discrete_control_torques[ui], next_state);
			temp_achievable_states[ui][0] = RRT_tree[nearest_state_index][0] + time_step * next_state[0];
			temp_achievable_states[ui][1] = RRT_tree[nearest_state_index][1] + time_step * next_state[1];
		}

		// select the closest reachable state point
		euclidianDistSquare(random_state, temp_achievable_states, number_of_discrete_torques, distance_square_values);
		ui = findMin(distance_square_values, number_of_discrete_torques);
		random_state[0] = temp_achievable_states[ui][0];
		random_state[1] = temp_achievable_states[ui][1];

		// if angular position is greater than pi rads, wrap around
		if (random_state[0] > M_PI || random_state[0] < -M_PI)
			random_state[0] = fmod((random_state[0] + M_PI), (2 * M_PI)) - M_PI;

		// link reachable state point to the nearest vertex in the tree
		RRT_tree[iteration][0] = random_state[0];
		RRT_tree[iteration][1] = random_state[1];
		parent_state_index[iteration] = nearest_state_index;
		control_action_index[iteration] = ui;

		// if tree has grown near enough to one of the surrounding goal states
		// set that particular goal state to 'found'
		// save path from initial state to that goal state
		for (goal_index = 0; goal_index < NUM_OF_GOAL_STATES; goal_index++) {
			if (not_found[goal_index] == 1
					&& (random_state[0] <= end_state[goal_index][0] + 0.05)
					&& (random_state[0] >= end_state[goal_index][0] - 0.05)) {
				if ((random_state[1] <= end_state[goal_index][1] + 0.25)
						&& (random_state[1] >= end_state[goal_index][1] - 0.25)) {

					not_found[goal_index] = 0;
					state_index = iteration;
					int length_of_soln = 0;
					while (state_index != 0) {
						u_path[goal_index][length_of_soln] = discrete_control_torques[control_action_index[state_index]];
						x_path[goal_index][length_of_soln][0] = RRT_tree[state_index][0];
						x_path[goal_index][length_of_soln][1] = RRT_tree[state_index][1];
						length_of_soln++;

						state_index = parent_state_index[state_index];
					}
				}
			}
		}
	}


	// Update adjacency matrix:
	// for each goal state surrounding an initial state,
	// if the goal state has been reached,
	// if tree is growing near border of phase space, check if tree is growing within state space limits
	// set respective flag in adjacency matrix to 1 (or to a weight)
	//*
	int offset[8] = {-43,-42,-41,-1,1,41,42,43};
	int offset_idx = 0;
	weight = 1;
	int k;
	for (k = 0; k < NUM_OF_GOAL_STATES; k++) {
		if (not_found[k] == 0) {
			offset_idx = offset[k];
			if((idx * NUM_THREADS * NUM_BLOCKS + idx + offset_idx >= 0) && (idx * NUM_THREADS * NUM_BLOCKS + idx + offset_idx < NUM_RESULTS_PER_THREAD * NUM_THREADS * NUM_BLOCKS)){
				if((end_state[k][0] > ANG_POS_MIN+DELTA_X) && (end_state[k][0] < ANG_POS_MAX-DELTA_X) &&
						(end_state[k][1] > ANG_VEL_MIN+DELTA_Y) && (end_state[k][1] < ANG_VEL_MAX-DELTA_Y) ){
					adjacency_matrix[idx * NUM_THREADS * NUM_BLOCKS + idx + offset_idx] = weight;
				}
			}
		}
	}
	//*/

	//* copy path results of algorithm to device results array
	int i, j;
	int num_of_goals = NUM_OF_GOAL_STATES;
	for (j = 0; j < num_of_goals; j++) {
		for (i = 0; i < LENGTH_OF_SOLN_PATH; i++) {
			path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i] = x_path[j][i][0];
			path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i + 1] = x_path[j][i][1];
			control_solutions[idx * num_of_goals * LENGTH_OF_SOLN_PATH + j * LENGTH_OF_SOLN_PATH + i] = u_path[j][i];
			if (not_found[j] == 0) {
				if (i == LENGTH_OF_SOLN_PATH - 2) {
					path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i] = start_state[0];
					path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i + 1] = start_state[1];
				} else if (i == LENGTH_OF_SOLN_PATH - 1) {
					path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i] = end_state[j][0];
					path_solutions[idx * DIMENSIONS * num_of_goals * LENGTH_OF_SOLN_PATH + j * DIMENSIONS * LENGTH_OF_SOLN_PATH + DIMENSIONS * i + 1] = end_state[j][1];
				}
			}
		}
	}
	//*/


	/*
	 int i;
	 for (i = 0; i < NUM_RESULTS_PER_THREAD; i++)
	 result[idx * NUM_RESULTS_PER_THREAD + i] = start_state[i];
	 //*/
	/*
	 result[idx * NUM_RESULTS_PER_THREAD + 0] = start_state[0];
	 result[idx * NUM_RESULTS_PER_THREAD + 1] = start_state[1];
	 //*/

}



////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////
/*
 * computes the Euclidian distances squared from point A to every point in array B
 */
__device__ void euclidianDistSquare(double* A, double B[][2], int lengthOfB,
		double* listOfDistSq) {
	int i;
	for (i = 0; i < lengthOfB; i++)
		listOfDistSq[i] = pow((B[i][0] - A[0]), 2) + pow((B[i][1] - A[1]), 2);
}

/*
 * finds the index of the minimum in an array
 */
__device__ int findMin(double array[], int lengthOfArray) {
	int minIndex = 0;

	int i;
	for (i = 0; i < lengthOfArray; i++) {
		if (array[i] < array[minIndex])
			minIndex = i;
	}

	return minIndex;
}

/*
 * Computes x_dot of the pendulum, given x and a control input u
 */
__device__ void pendulumDynamics(double* x, double u, double* next_state) {
	// pendulum parameters
	int m = 1;                  // mass
	int l = 1;                  // length of pendulum link
	int I = m * l * l;              // moment of inertia
	double g = 9.8;              // acceleration due to gravity
	double b = 0.1;              // damping factor

	next_state[0] = x[1];
	next_state[1] = (u - m * g * l * sin((M_PI / 2) - x[0]) - b * x[1]) / I;
}
