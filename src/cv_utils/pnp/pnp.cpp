#include <code_utils/cv_utils/pnp/pnp.h>

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
{

    //      cv::LinearPnP* lpnp = new cv::LinearPnP( image_point, scene_point );
    cv::LinearPnP llpnp( image_point, scene_point );

    //    std::cout << "P_2 " << std::endl << llpnp.getT( ).transpose( ) << std::endl;

    cv::NonlinearPnP nlpnp( llpnp.getR( ), llpnp.getT( ), image_point, scene_point );
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
