#include <code_utils/cv_utils/pnp/pnp.h>

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
{

    bool is_planar = false;
    if ( scene_point.size( ) < 4 )
    {
        is_planar = true;
    }
    else
    {
        int size = scene_point.size( );
        Eigen::MatrixXd scene_points( size, 3 );
        for ( int index = 0; index < size; ++index )
        {
            scene_points( index, 0 ) = scene_point[index]( 0 );
            scene_points( index, 1 ) = scene_point[index]( 1 );
            scene_points( index, 2 ) = scene_point[index]( 2 );
        }
        Eigen::JacobiSVD< Eigen::MatrixXd > svd( scene_points, Eigen::EigenvaluesOnly );
        if ( svd.singularValues( )( 2 ) / svd.singularValues( )( 1 ) < 1e-3 )
            is_planar = true;
        else
            is_planar = false;
    }

    Eigen::Matrix3d R_cw;
    Eigen::Vector3d t_cw;
    if ( is_planar )
    {
        cv::LinearPnP llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }
    else
    {
        cv::DLT llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }

    std::cout << "P_2 " << std::endl << t_cw.transpose( ) << std::endl;

    cv::NonlinearPnP nlpnp( R_cw, t_cw, image_point, scene_point );
    nlpnp.getRT( T_dst, q_dst );
}

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              const Eigen::Quaterniond& q_init,
              const Eigen::Vector3d& T_init,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
{
    cv::NonlinearPnP nlpnp( q_init.toRotationMatrix( ), T_init, image_point, scene_point );
    nlpnp.getRT( T_dst, q_dst );
}
