/**
 * iso surface map from Kim's code
 * @author: Kanzhi Wu
 */


#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <cmath>
#include <algorithm>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "include/iso_surface.hpp"

using namespace std;


//const float         Epsilon<float>::value       = numeric_limits<float>::epsilon();
//const double        Epsilon<double>::value			= numeric_limits<double>::epsilon();
//const long double   Epsilon<long double>::value	= numeric_limits<long double>::epsilon();



int main( int argc, char ** argv ) {
  IsoSurfaceExtraction ise(0.002);
  return 1;
}
