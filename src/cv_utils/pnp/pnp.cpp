#include <code_utils/cv_utils/pnp/pnp.h>

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
{
    cv::LinearPnP m_pnp( image_point, scene_point );

    std::cout << "------------------------ " << std::endl;
    //    std::cout << "R_2 " << std::endl << m_pnp.getR( ) << std::endl;
    std::cout << "P_2 " << std::endl << m_pnp.getT( ).transpose( ) << std::endl;

    cv::NonlinearPnP m_npnp( m_pnp.getR( ), m_pnp.getT( ), image_point, scene_point );
    m_npnp.getRT( T_dst, q_dst );
}

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              const Eigen::Quaterniond& q_init,
              const Eigen::Vector3d& T_init,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
{
    cv::NonlinearPnP m_npnp( q_init.toRotationMatrix( ), T_init, image_point, scene_point );
    m_npnp.getRT( T_dst, q_dst );
}
