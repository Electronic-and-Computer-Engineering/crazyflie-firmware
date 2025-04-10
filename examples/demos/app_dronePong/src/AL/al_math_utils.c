#include "al_math_utils.h"
#include <math.h>

// Convert radians to degrees
float radToDeg(float rad) 
{
    return rad * (180.0f / (float)M_PI);
}

// Convert degrees to radians
float degToRad(float deg) 
{
    return deg * ((float)M_PI / 180.0f);
}