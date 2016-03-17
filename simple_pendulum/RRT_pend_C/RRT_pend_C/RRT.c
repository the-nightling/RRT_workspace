#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "RRT.h"

int RRT_kernel(double control_solution[])
{
    //    double start_state[] = {-M_PI/2,0};   // initial state; angle position measured from x-axis
    double end_state[] = {M_PI/2,0};    // goal state

    double state_limits[2][2] = {{-M_PI,M_PI},{-8,8}};  // state limits; angular position between -pi & pi rad; angular velocity between -10 & 10 rad/s

    // control torques to be used: linspace(-5,5,20)
    double discrete_control_torques[] = {-5.0000,-4.4737,-3.9474,-3.4211,-2.8947,-2.3684,-1.8421,-1.3158,-0.7895,-0.2632,
                5.0000, 4.4737, 3.9474, 3.4211, 2.8947, 2.3684, 1.8421, 1.3158, 0.7895, 0.2632};
    int number_of_discrete_torques = (int)( sizeof(discrete_control_torques)/sizeof(discrete_control_torques[0]) );

    double time_step = 0.02;            // time interval between application of subsequent control torques

    // static memory allocation
    double random_state[2];        // stores a state
    double next_state[2];
    //double RRT_tree[50000][2] = { [0 ... 50000-1] = {-M_PI/2,0} }; // graph of states in RRT; each index corresponds to a vertex (using designated initializer)
    double RRT_tree[NUM_OF_ITERATIONS][2];

    int x;
    for(x = 0; x < NUM_OF_ITERATIONS; x++)
    {
        RRT_tree[x][0] = -M_PI/2;
        RRT_tree[x][1] = 0;
    }

    int parent_state_index[NUM_OF_ITERATIONS];         // stores index of parent state for each state in graph RRT_tree
    int control_action_index[NUM_OF_ITERATIONS];        // stores index of control actions in discrete_control_torques (each state will use a control action value in discrete_control_torques)

    int state_index = 0;    // stores sequence of states joining initial to goal state
    double temp_achievable_states[number_of_discrete_torques][2]; // stores temporary achievable states from a particular vertex

    double distance_square_values[NUM_OF_ITERATIONS];  // stores distance square values

    srand(time(NULL));  // initialize random number generator


    // keep growing RRT until goal found or run out of iterations
    int iteration;
    for(iteration = 1; iteration < NUM_OF_ITERATIONS; iteration++)
    {
        // get random state
        random_state[0] = generateRandomDouble(state_limits[0][0],state_limits[0][1]);
        random_state[1] = generateRandomDouble(state_limits[1][0],state_limits[1][1]);

        // find distances between that state point and every vertex in RRT
        euclidianDistSquare(random_state,RRT_tree,iteration,distance_square_values);

        // select RRT vertex closest to the state point
        int minIndex = findMin(distance_square_values,iteration);

        // from the closest RRT vertex, compute all the states that can be reached,
        // given the pendulum dynamics and available torques
        int ui;
        for(ui = 0; ui < number_of_discrete_torques; ui++)
        {
            pendulumDynamics(RRT_tree[minIndex],discrete_control_torques[ui],next_state);
            temp_achievable_states[ui][0] = RRT_tree[minIndex][0] + time_step*next_state[0];
            temp_achievable_states[ui][1] = RRT_tree[minIndex][1] + time_step*next_state[1];
        }

        // select the closest reachable state point
        euclidianDistSquare(random_state,temp_achievable_states,number_of_discrete_torques,distance_square_values);
        ui = findMin(distance_square_values,number_of_discrete_torques);
        random_state[0] = temp_achievable_states[ui][0];
        random_state[1] = temp_achievable_states[ui][1];

        // if angular position is greater than pi rads, wrap around
        if(random_state[0] > M_PI || random_state[0] < -M_PI)
            random_state[0] = fmod((random_state[0]+M_PI), (2*M_PI)) - M_PI;

        // link reachable state point to the nearest vertex in the tree
        RRT_tree[iteration][0] = random_state[0];
        RRT_tree[iteration][1] = random_state[1];
        parent_state_index[iteration] = minIndex;
        control_action_index[iteration] = ui;

        if( (random_state[0] <= end_state[0]+0.05) && (random_state[0] >= end_state[0]-0.05) )
        {
            if( (random_state[1] <= end_state[1]+0.25) && (random_state[1] >= end_state[1]-0.25) )
            {
                break;
            }
        }

    }

    if(iteration == NUM_OF_ITERATIONS)
    {
        printf("Number of iterations: %d\n",iteration);

        return 0;

    } else
    {
        printf("Number of iterations: %d\n",iteration);

        state_index = iteration;
        int length_of_soln = 0;
        while(state_index != 0)
        {
            control_solution[length_of_soln] = discrete_control_torques[ control_action_index[state_index] ];
            length_of_soln++;

            state_index = parent_state_index[state_index];
        }

        return length_of_soln;
    }
}

/*
 * generates a random double between the limits min and max
 */
double generateRandomDouble(double min, double max)
{
    return ( ((double)rand() / (double) RAND_MAX) * (max - min) ) + min;
}

/*
 * computes the euclidian distances squared from point A to every point in array B
 */
void euclidianDistSquare(double* A, double B[][2], int lengthOfB, double* listOfDistSq)
{
    int i;
    for(i = 0; i < lengthOfB; i++)
        listOfDistSq[i] = pow((B[i][0] - A[0]),2) + pow((B[i][1] - A[1]),2);
}

/*
 * finds the index of the minimum in an array
 */
int findMin(double array[], int lengthOfArray)
{
    int minIndex = 0;

    int i;
    for(i = 0; i < lengthOfArray; i++)
    {
        if(array[i] < array[minIndex])
            minIndex = i;
    }

    return minIndex;
}

/*
 * Computes x_dot of the pendulum, given x and a control input u
 */
void pendulumDynamics(double* x, double u, double* next_state)
{
    // pendulum parameters
    int m = 1;                  // mass
    int l = 1;                  // length of pendulum link
    int I = m*l*l;              // moment of inertia
    double g = 9.8;              // acceleration due to gravity
    double b = 0.1;              // damping factor

    next_state[0] = x[1];
    next_state[1] = (u - m*g*l*sin((M_PI/2)-x[0]) - b*x[1]) / I;
}

