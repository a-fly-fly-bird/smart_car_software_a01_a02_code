#include "algorithm_set/distance.h"
#include <math.h>

using namespace std;

namespace algorithm_set
{
    float CalculatingDistanceByAverageSpeed(int frequency, float speed_limit, float speed_x, float speed_y, float speed_z)
    {
        if (fabs(speed_x < speed_limit))
        {
            speed_x = 0;
        }
        if (fabs(speed_y < speed_limit))
        {
            speed_y = 0;
        }
        if (fabs(speed_z < speed_limit))
        {
            speed_z = 0;
        }
        return sqrt(speed_x * speed_x + speed_y * speed_y + speed_z * speed_z) / frequency;
    }
}
