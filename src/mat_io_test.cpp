#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include <code_utils/cv_utils/cv_type.hpp>
#include <code_utils/sys_utils/cvmat_file_io.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;
using namespace cv;

double t;

void
showImg( std::string name, cv::Mat img )
{
    cv::namedWindow( name, WINDOW_NORMAL );
    cv::imshow( name, img );
}

int
main( )
{
    Mat img;

    //    Mat img1 = imread( "/home/gao/ws/devel/lib/camera_model/image_down/IMG_35.png",
    //    CV_LOAD_IMAGE_GRAYSCALE );
    Mat img1 = imread( "/home/gao/IMG_1.png", CV_LOAD_IMAGE_UNCHANGED );

    cv::resize( img1, img, cv::Size( 640, 512 ) );

    sys_utils::io::writeMatrixToBinary( "/home/gao/img_data", img1 );

    cv::Mat img11;
    sys_utils::io::parseMatrixFromBinary( "/home/gao/img_data", img11 );
    showImg( "img_data", img11 );

    cv::Mat img2( 2, 2, CV_8UC12 );
    for ( int i = 0; i < 12; ++i )
    {
        img2.at< cv::Vec12b >( 0, 0 )[i] = uchar( 1 );
        img2.at< cv::Vec12b >( 0, 1 )[i] = uchar( 2 );
        img2.at< cv::Vec12b >( 1, 0 )[i] = uchar( 3 );
        img2.at< cv::Vec12b >( 1, 1 )[i] = uchar( 4 );
    }
    std::cout << img2 << "\n\n";
    sys_utils::io::writeMatrixToBinary( "/home/gao/img_data2", img2 );

    cv::Mat img22;
    sys_utils::io::parseMatrixFromBinary( "/home/gao/img_data2", img22 );
    std::cout << img22 << "\n\n";

    cv::Mat img33;
    sys_utils::io::parseMatrixFromBinary( "/home/gao/data_SAGAST12d_down", img33 );
    std::cout << img33 << "\n\n";

    waitKey( 0 );

    return 0;
}
