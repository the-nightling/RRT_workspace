#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "kdtree.h"

/* get a random double between -8 and 8 */
static double rd( void );

int main(void)
{
    void *kd_tree;
    double position[2], test_position[2];
    struct kdres *kd_results;

    srand( time(0) );

    kd_tree = kd_create(2);

    data = malloc(10);
    int i;
    for( i=0; i<10; i++ ) {
      position[0] = rd();
      position[1] = rd();
      kd_insert(kd_tree, position, NULL);
    }

    test_position[0] = 5.0;
    test_position[1] = 6.0;
    kd_results = kd_nearest(kd_tree, test_position);

    kd_res_item(kd_results, position);

    printf("Position: %f %f\n", position[0], position[1]);

    return 0;
}

static double rd( void ) {
  return (double)rand()/RAND_MAX * 20.0 - 10.0;
}
