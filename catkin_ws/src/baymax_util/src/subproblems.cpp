#include <iostream>

#include "baymax_util/subproblems.hpp"

Subproblem::Calc_Result Subproblem::Prob_1( cv::Point3d k, cv::Point3d u, cv::Point3d v, double& theta )
{
    if( cv::norm( k ) < DBL_EPSILON ) return NO_SOLUTION;
    if( cv::norm( u - k ) < DBL_EPSILON ) return NO_SOLUTION;
    if( fabs( cv::norm( u ) - cv::norm( v ) ) > DBL_EPSILON ) return NO_SOLUTION;
    if( fabs( k.dot( u ) - k.dot( v ) ) > DBL_EPSILON ) return NO_SOLUTION;
    k = ( 1 / cv::norm( k ) ) * k;

    cv::Point3d up = u - k.dot( u ) * k;
    cv::Point3d vp = v - k.dot( v ) * k;

    theta = atan2(
        k.dot( up.cross( vp ) ),
        up.dot( vp )
    );
    return ONE_SOLUTION;
}

Subproblem::Calc_Result Subproblem::Prob_2( cv::Point3d k1, cv::Point3d k2, cv::Point3d u, cv::Point3d v, double& theta1, double& theta2 )
{
    if( cv::norm( k1 ) < DBL_EPSILON ) return NO_SOLUTION;
    if( cv::norm( k2 ) < DBL_EPSILON ) return NO_SOLUTION;
    if( fabs( k1.dot( k2 ) - 1 ) < DBL_EPSILON ) return NO_SOLUTION;
    if( fabs( cv::norm( u ) - cv::norm( v ) ) > DBL_EPSILON ) return NO_SOLUTION;
    k1 = ( 1 / cv::norm( k1 ) ) * k1;
    k2 = ( 1 / cv::norm( k2 ) ) * k2;
}

int main()
{
    double t;
    assert( Subproblem::Prob_1( cv::Point3d( 0.0, 0.0, 1 ), cv::Point3d( 0.0, 0.707, 0.0 ), cv::Point3d( 0.0, -0.707, 0.0 ), t ) == Subproblem::ONE_SOLUTION );
    std::cout << t << std::endl;
    return 0;
}
