#include <code_utils/cv_utils/pnp/nonlinearpnp.h>

cv::NonlinearPnP::NonlinearPnP( const Eigen::Matrix3d& _R_initial,
                                const Eigen::Vector3d& _T_initial,
                                const std::vector< Eigen::Vector3d >& image_point,
                                const std::vector< Eigen::Vector3d >& scene_point )
: T( _T_initial )
{
    if ( image_point.size( ) != scene_point.size( ) )
        std::cerr << "Error of point size" << std::endl;
    q = _R_initial;

    // initialize the params to something close to the gt
    double ext[] = { q.w( ), q.x( ), q.y( ), q.z( ), T[0], T[1], T[2] };

    point_num = image_point.size( );

    ceres::Problem problem;

    for ( int i = 0; i < point_num; ++i )
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction< ReprojectionError, 2, 7 >(
        new ReprojectionError( image_point[i], scene_point[i] ) );

        problem.AddResidualBlock( costFunction, NULL /* squared loss */, ext );
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type   = ceres::DOGLEG;
    //    options.max_num_iterations           = 20;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    //    std::cout << summary.FullReport( ) << "\n";

    q.w( ) = ext[0];
    q.x( ) = ext[1];
    q.y( ) = ext[2];
    q.z( ) = ext[3];
    T << ext[4], ext[5], ext[6];
}

bool
cv::NonlinearPnP::getRT( Eigen::Vector3d& T_out, Eigen::Quaterniond& q_out )
{
    q_out = q.normalized( );
    T_out = T;
    return true;
}

cv::NonlinearPnP::ReprojectionError::ReprojectionError( const Eigen::Vector3d& image_point,
                                                        const Eigen::Vector3d& scene_point )
: image_point_( image_point )
, scene_point_( scene_point )
{
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d tmp( 0, 0, 1 );
    if ( image_point_ == tmp )
        tmp << 1, 0, 0;
    b1 = ( tmp - image_point_ * ( image_point_.transpose( ) * tmp ) ).normalized( );
    b2 = image_point_.cross( b1 );
    tangent_base.block< 1, 3 >( 0, 0 ) = b1.transpose( );
    tangent_base.block< 1, 3 >( 1, 0 ) = b2.transpose( );
}
