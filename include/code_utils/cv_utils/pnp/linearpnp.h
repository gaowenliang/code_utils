#ifndef LINEARPNP_H
#define LINEARPNP_H

#include "utils.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace cv
{
class LinearPnP
{
    public:
    LinearPnP( const std::vector< Eigen::Vector3d >& pts_2, const std::vector< Eigen::Vector3d >& pts_3 );
    //    ~LinearPnP( ) {}

    public:
    Eigen::Matrix3d getR( ) const { return R; }
    Eigen::Vector3d getT( ) const { return T; }

    private:
    void readPointsPlanar( const std::vector< Eigen::Vector3d >& pts_2,
                           const std::vector< Eigen::Vector3d >& pts_3 );
    void solvePnP( );

    private:
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

    Eigen::MatrixXd M;
};
}

#endif // LINEARPNP_H
