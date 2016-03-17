#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "kdtree.h"

#define NUM_OF_ITERATIONS 10

/* get a random double between -8 and 8 */
static double rd( void );

int main(void)
{
    void *kd_tree;
    double position[2], test_position[2];
    struct kdres *kd_results;
    int data[NUM_OF_ITERATIONS];
    int * ans;

    srand( time(0) );

    kd_tree = kd_create(2);

    int i;
    for( i=0; i<NUM_OF_ITERATIONS; i++ ) {
        data[i] = i;
        position[0] = rd();
        position[1] = rd();
        kd_insert(kd_tree, position, &data[i]);
    }

    test_position[0] = 5.0;
    test_position[1] = 6.0;
    kd_results = kd_nearest(kd_tree, test_position);

    ans = (int*) kd_res_item(kd_results, position);

    printf("Position: %f %f with index %d\n", position[0], position[1], *ans);

    return 0;
}

static double rd( void ) {
    return (double)rand()/RAND_MAX * 20.0 - 10.0;
}
