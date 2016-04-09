#include <stdio.h>
#include <math.h>

#define NUM_BLOCKS 8
#define NUM_OF_GOAL_STATES 8
#define ANG_POS_MIN -M_PI		// minimum angular position
#define ANG_POS_MAX M_PI		// maximum angular position
#define ANG_VEL_MIN -8.0		// minimum angular velocity
#define ANG_VEL_MAX 8.0		// maximum angular velocity
#define GRID_X 42
#define GRID_Y 6
#define DELTA_X (ANG_POS_MAX-ANG_POS_MIN)/(2*GRID_X + 4)
#define DELTA_Y (ANG_VEL_MAX-ANG_VEL_MIN)/(2*GRID_Y*NUM_BLOCKS + 4)

int main(void)
{
    int idx;
    for(idx=0; idx < 50; idx++){
        double start_state[] = { ANG_POS_MIN+2*DELTA_X, ANG_VEL_MIN+2*DELTA_Y }; // initial state; angle position measured from x-axis
        start_state[0] += ((idx % GRID_X) * 2 * DELTA_X) + (2 * DELTA_X);
        start_state[1] += ((idx / GRID_X) * 2 * DELTA_Y) + (2 * DELTA_Y);

        double end_state[NUM_OF_GOAL_STATES][2] = {{0}};
        int goal_idx;
        for(goal_idx = 0; goal_idx < pow((float)3,(float)2); goal_idx++)
        {
            end_state[goal_idx][0] = start_state[0] + ((goal_idx%3) - 1)*2*DELTA_X;
            end_state[goal_idx][1] = start_state[1] + (((goal_idx/3)%3) - 1)*2*DELTA_Y;
        }

        double state_limits[2][2] = { { start_state[0] - 3 * DELTA_X, start_state[0] + 3 * DELTA_X }, {
                                          start_state[1] - 3 * DELTA_Y, start_state[1] + 3 * DELTA_Y } }; // state limits; angular position between -pi & pi rad; angular velocity between -10 & 10 rad/s



        printf("idx : %d\n", idx);
        printf("start_state : %f, %f\n",start_state[0],start_state[1]);
        for(goal_idx=0; goal_idx < 9; goal_idx++)
            printf("end_state%d : %f, %f\n", goal_idx, end_state[goal_idx][0], end_state[goal_idx][0]);
        printf("state_limits : %f, %f, %f, %f\n\n", state_limits[0][0], state_limits[0][1], state_limits[1][0], state_limits[1][1]);
        printf("state_limits : %f, %f, %f, %f\n\n", ANG_POS_MIN, ANG_POS_MAX, ANG_VEL_MIN, ANG_VEL_MAX);
    }


    return 0;
}

