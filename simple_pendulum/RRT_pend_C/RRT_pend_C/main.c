/*
 * Computes the sequence of control actions needed for the swing up of
 * a simple pendulum using a Rapidly Exploring Random Tree.
 * The tree will grow from the initial state and span the phase space area,
 * looking for the goal point/state.
 *
 * Author: Nirav Domah
 * Date: 16/03/16
 */

/////////////////////////////////////////////////////////////////
// INCLUDES
/////////////////////////////////////////////////////////////////

// includes from C
#include <stdio.h>
#include <stdlib.h>
#include "RRT.h"

/////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////

int main(void)
{
    double control_solution[1000]; // stores sequence of control actions (solution to problem)
    int length_of_soln;

    length_of_soln = RRT_kernel(control_solution);

    if(length_of_soln)
    {
        printf("Simulation complete (goal found)\n");

        FILE *dataFile = fopen("data.txt", "w");

        while(length_of_soln > 0)
        {
            fprintf(dataFile, "%f\n",control_solution[length_of_soln-1]);
            length_of_soln--;
        }

        fclose(dataFile);
    } else {
        printf("Simulation complete (goal not found; ran out of iterations)\n");
    }

    return 0;
}


