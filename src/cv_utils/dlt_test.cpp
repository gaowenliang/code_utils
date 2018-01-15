//! \example pose-dlt-opencv.cpp

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

//! [Include]
#include <code_utils/cv_utils/dlt/dlt.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

//! [Estimation function]
void
pose_dlt_eigen( const std::vector< cv::Point3d >& wX, const std::vector< cv::Point2d >& x, cv::Mat& ctw, cv::Mat& cRw )
//! [Estimation function]
{
    //! [DLT]
    int npoints = ( int )wX.size( );
    //    cv::Mat A( 2 * npoints, 12, CV_64F, cv::Scalar( 0 ) );

    Eigen::MatrixXd M( 2 * npoints, 12 );
    //    M.setZero( );
    for ( int i = 0; i < npoints; i++ )
    { // Update matrix A using eq. 5
        M( 2 * i, 0 ) = wX[i].x;
        M( 2 * i, 1 ) = wX[i].y;
        M( 2 * i, 2 ) = wX[i].z;
        M( 2 * i, 3 ) = 1;

        M( 2 * i + 1, 0 ) = 0;
        M( 2 * i + 1, 1 ) = 0;
        M( 2 * i + 1, 2 ) = 0;
        M( 2 * i + 1, 3 ) = 0;

        M( 2 * i, 4 ) = 0;
        M( 2 * i, 5 ) = 0;
        M( 2 * i, 6 ) = 0;
        M( 2 * i, 7 ) = 0;

        M( 2 * i + 1, 4 ) = wX[i].x;
        M( 2 * i + 1, 5 ) = wX[i].y;
        M( 2 * i + 1, 6 ) = wX[i].z;
        M( 2 * i + 1, 7 ) = 1;

        M( 2 * i, 8 )  = -x[i].x * wX[i].x;
        M( 2 * i, 9 )  = -x[i].x * wX[i].y;
        M( 2 * i, 10 ) = -x[i].x * wX[i].z;
        M( 2 * i, 11 ) = -x[i].x;

        M( 2 * i + 1, 8 )  = -x[i].y * wX[i].x;
        M( 2 * i + 1, 9 )  = -x[i].y * wX[i].y;
        M( 2 * i + 1, 10 ) = -x[i].y * wX[i].z;
        M( 2 * i + 1, 11 ) = -x[i].y;
    }

    std::cout << " M " << std::endl << M << std::endl;

    Eigen::JacobiSVD< Eigen::MatrixXd > svd( M, Eigen::ComputeThinV );
    std::cout << " svd.matrixV( ) " << std::endl << svd.matrixV( ) << std::endl;

    Eigen::MatrixXd h = svd.matrixV( ).block< 12, 1 >( 0, 11 ).transpose( );

    std::cout << " h " << std::endl << h << std::endl;

    if ( h( 0, 11 ) < 0 ) // tz < 0
        h *= -1;

    double norm = sqrt( h( 0, 8 ) * h( 0, 8 ) + h( 0, 9 ) * h( 0, 9 ) + h( 0, 10 ) * h( 0, 10 ) );

    h /= norm;

    for ( int i = 0; i < 3; i++ )
    {
        ctw.at< double >( i, 0 ) = h( 0, 4 * i + 3 ); // Translation

        for ( int j = 0; j < 3; j++ )
            cRw.at< double >( i, j ) = h( 0, 4 * i + j ); // Rotation
    }
}

void
pose_dlt( const std::vector< cv::Point3d >& wX, const std::vector< cv::Point2d >& x, cv::Mat& ctw, cv::Mat& cRw )
//! [Estimation function]
{
    //! [DLT]
    int npoints = ( int )wX.size( );
    cv::Mat A( 2 * npoints, 12, CV_64F, cv::Scalar( 0 ) );
    for ( int i = 0; i < npoints; i++ )
    {
        // Update matrix A using eq. 5
        A.at< double >( 2 * i, 0 ) = wX[i].x;
        A.at< double >( 2 * i, 1 ) = wX[i].y;
        A.at< double >( 2 * i, 2 ) = wX[i].z;
        A.at< double >( 2 * i, 3 ) = 1;

        A.at< double >( 2 * i + 1, 4 ) = wX[i].x;
        A.at< double >( 2 * i + 1, 5 ) = wX[i].y;
        A.at< double >( 2 * i + 1, 6 ) = wX[i].z;
        A.at< double >( 2 * i + 1, 7 ) = 1;

        A.at< double >( 2 * i, 8 )  = -x[i].x * wX[i].x;
        A.at< double >( 2 * i, 9 )  = -x[i].x * wX[i].y;
        A.at< double >( 2 * i, 10 ) = -x[i].x * wX[i].z;
        A.at< double >( 2 * i, 11 ) = -x[i].x;

        A.at< double >( 2 * i + 1, 8 )  = -x[i].y * wX[i].x;
        A.at< double >( 2 * i + 1, 9 )  = -x[i].y * wX[i].y;
        A.at< double >( 2 * i + 1, 10 ) = -x[i].y * wX[i].z;
        A.at< double >( 2 * i + 1, 11 ) = -x[i].y;
    }

    //    std::cout << " A " << std::endl << A << std::endl;

    cv::Mat w, u, vt;
    cv::SVD::compute( A, w, u, vt );
    //    std::cout << " svd.matrixV( ) " << std::endl << vt << std::endl;

    double smallestSv            = w.at< double >( 0, 0 );
    unsigned int indexSmallestSv = 0;
    for ( int i = 1; i < w.rows; i++ )
    {
        if ( ( w.at< double >( i, 0 ) < smallestSv ) )
        {
            smallestSv      = w.at< double >( i, 0 );
            indexSmallestSv = i;
        }
    }
    cv::Mat h = vt.row( indexSmallestSv );

    //    std::cout << " h " << std::endl << h << std::endl;

    if ( h.at< double >( 0, 11 ) < 0 ) // tz < 0
        h *= -1;

    // Normalization to ensure that ||r3|| = 1
    double norm = sqrt( h.at< double >( 0, 8 ) * h.at< double >( 0, 8 )
                        + h.at< double >( 0, 9 ) * h.at< double >( 0, 9 )
                        + h.at< double >( 0, 10 ) * h.at< double >( 0, 10 ) );

    h /= norm;
    //! [DLT]

    //! [Update homogeneous matrix]
    for ( int i = 0; i < 3; i++ )
    {
        ctw.at< double >( i, 0 ) = h.at< double >( 0, 4 * i + 3 ); // Translation
        for ( int j = 0; j < 3; j++ )
            cRw.at< double >( i, j ) = h.at< double >( 0, 4 * i + j ); // Rotation
    }
    //! [Update homogeneous matrix]
}

//! [Main function]
int
main( )
//! [Main function]
{
    //! [Create data structures]
    std::vector< cv::Point3d > wX;
    std::vector< cv::Point2d > x;

    std::vector< Eigen::Vector3d > wX_tmp;
    std::vector< Eigen::Vector3d > x_tmp;
    //! [Create data structures]

    //! [Simulation]
    // Ground truth pose used to generate the data
    cv::Mat ctw_truth = ( cv::Mat_< double >( 3, 1 ) << -0.1, 0.1, 1.2 ); // Translation vector
    cv::Mat crw_truth
    = ( cv::Mat_< double >( 3, 1 ) << CV_PI / 180 * ( 5 ), CV_PI / 180 * ( 0 ), CV_PI / 180 * ( 45 ) ); // Rotation vector
    cv::Mat cRw_truth( 3, 3, cv::DataType< double >::type ); // Rotation matrix
    cv::Rodrigues( crw_truth, cRw_truth );

    // Input data: 3D coordinates of at least 6 non coplanar points
    double L = 0.2;
    wX.push_back( cv::Point3d( -L, -L, 0 ) );      // wX_0 ( -L, -L, 0  )^T
    wX.push_back( cv::Point3d( 2 * L, -L, 0.2 ) ); // wX_1 (-2L, -L, 0.2)^T
    wX.push_back( cv::Point3d( L, L, 0.2 ) );      // wX_2 (  L,  L, 0.2)^T
    wX.push_back( cv::Point3d( -L, L, 0 ) );       // wX_3 ( -L,  L, 0  )^T
    wX.push_back( cv::Point3d( -2 * L, L, 0 ) );   // wX_4 (-2L,  L, 0  )^T
    wX.push_back( cv::Point3d( 0, 0, 0.5 ) );      // wX_5 (  0,  0, 0.5)^T

    wX_tmp.push_back( Eigen::Vector3d( -L, -L, 0 ) );
    wX_tmp.push_back( Eigen::Vector3d( 2 * L, -L, 0.2 ) );
    wX_tmp.push_back( Eigen::Vector3d( L, L, 0.2 ) );
    wX_tmp.push_back( Eigen::Vector3d( -L, L, 0 ) );
    wX_tmp.push_back( Eigen::Vector3d( -2 * L, L, 0 ) );
    wX_tmp.push_back( Eigen::Vector3d( 0, 0, 0.5 ) );

    // Input data: 2D coordinates of the points on the image plane
    for ( int i = 0; i < wX.size( ); i++ )
    {
        cv::Mat cX = cRw_truth * cv::Mat( wX[i] ) + ctw_truth; // Update cX, cY, cZ
        x.push_back(
        cv::Point2d( cX.at< double >( 0, 0 ) / cX.at< double >( 2, 0 ),
                     cX.at< double >( 1, 0 ) / cX.at< double >( 2, 0 ) ) ); // x = (cX/cZ, cY/cZ)

        x_tmp.push_back( Eigen::Vector3d( cX.at< double >( 0, 0 ), //
                                          cX.at< double >( 1, 0 ),
                                          cX.at< double >( 2, 0 ) ) );
    }
    //! [Simulation]

    //! [Call function]
    cv::Mat ctw( 3, 1, CV_64F ); // Translation vector
    cv::Mat cRw( 3, 3, CV_64F ); // Rotation matrix

    std::cout << " _________________________________ " << std::endl;
    pose_dlt( wX, x, ctw, cRw );

    std::cout << "ctw (ground truth):\n" << ctw_truth << std::endl;
    std::cout << "ctw (computed with DLT):\n" << ctw << std::endl;
    std::cout << "cRw (ground truth):\n" << cRw_truth << std::endl;
    std::cout << "cRw (computed with DLT):\n" << cRw << std::endl;

    std::cout << " _________________________________ " << std::endl;
    pose_dlt_eigen( wX, x, ctw, cRw );

    std::cout << "ctw (ground truth):\n" << ctw_truth << std::endl;
    std::cout << "ctw (computed with DLT):\n" << ctw << std::endl;
    std::cout << "cRw (ground truth):\n" << cRw_truth << std::endl;
    std::cout << "cRw (computed with DLT):\n" << cRw << std::endl;

    std::cout << " _________________________________ " << std::endl;
    cv::DLT dlt( x_tmp, wX_tmp );
    std::cout << "cRw (computed with DLT):\n" << dlt.getR( ) << std::endl;
    std::cout << "ctw (computed with DLT):\n" << dlt.getT( ) << std::endl;

    //! [Call function]

    return 0;
}
