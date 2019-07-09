
#ifndef TRANSFORM_UTILITY
#define TRANSFORM_UTILITY

#include <Eigen/Dense>

/**function for conversation**/

using namespace Eigen;

Matrix3f euler2rot(const float x_pi,const float y_pi,const float z_pi){
    Matrix3f m;
    m = AngleAxisf(x_pi*M_PI, Vector3f::UnitX())
        * AngleAxisf(y_pi*M_PI, Vector3f::UnitY())
        * AngleAxisf(z_pi*M_PI, Vector3f::UnitZ());
    return m;
}
Vector3f rot2euler(const Matrix3f m){
    return m.eulerAngles(0,1,2);
}

#endif //TRANSFORM_UTILITY
