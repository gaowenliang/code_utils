#ifndef CVMAT_FILE_IO_HPP
#define CVMAT_FILE_IO_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace sys_utils
{

namespace io
{

void
writeMatrixToBinary( const std::string filename, const cv::Mat& matrix )
{
    std::ofstream out( filename, std::ios::out | std::ios::binary | std::ios::trunc );
    int rows = matrix.rows, cols = matrix.cols, step = matrix.step, type = matrix.type( );

    out.write( ( char* )( &rows ), sizeof( int ) );
    out.write( ( char* )( &cols ), sizeof( int ) );
    out.write( ( char* )( &step ), sizeof( int ) );
    out.write( ( char* )( &type ), sizeof( int ) );
    out.write( ( char* )matrix.data, rows * step * sizeof( uchar ) );

    out.close( );
}

void
parseMatrixFromBinary( const std::string filename, cv::Mat& matrix )
{
    std::ifstream in( filename, std::ios::in | std::ios::binary );
    while ( in.peek( ) != EOF )
    {
        int rows = 0, cols = 0, step = 0, type = 0;

        in.read( ( char* )( &rows ), sizeof( int ) );
        in.read( ( char* )( &cols ), sizeof( int ) );
        in.read( ( char* )( &step ), sizeof( int ) );
        in.read( ( char* )( &type ), sizeof( int ) );

        matrix = cv::Mat( rows, cols, type );

        in.read( ( char* )matrix.data, rows * step * sizeof( uchar ) );
    }

    in.close( );
}
}
}

#endif // CVMAT_FILE_IO_HPP
