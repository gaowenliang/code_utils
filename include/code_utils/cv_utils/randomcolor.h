#ifndef RANDOMCOLOR_H
#define RANDOMCOLOR_H

#include <opencv2/core/core.hpp>

namespace cv
{

class RandomColor
{
    public:
    RandomColor( ) { randColor( ); }

    void randColor( )
    {
        color0 = rand( ) % 256;
        color1 = rand( ) % 256;
        color2 = rand( ) % 256;
    }
    cv::Scalar getColor( ) { return cv::Scalar( color0, color1, color2 ); }

    private:
    int color0;
    int color1;
    int color2;
};
}

#endif // RANDOMCOLOR_H
