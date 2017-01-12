#include "parameters.h"
#include <fstream>

Parameters::Parameters()
{
}

void Parameters::readFromfile(std::string filename)
{
    // load intrinsics
    std::ifstream intrinsics_file;
    intrinsics_file.open(filename.c_str());
    std::string intrinsics_file_line;
    for(size_t i=0; i<3; ++i)
    {
        std::getline(intrinsics_file,intrinsics_file_line);

        std::stringstream str(intrinsics_file_line);
        str >> K_cam(i,0) >> K_cam(i,1) >> K_cam(i,2);
    }
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> output_size_x;
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> output_size_y;
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> radial;
    intrinsics_file.close();

    K_caminv = K_cam.inverse();
    px = output_size_x/2.f;
    py = output_size_y/2.f;
}
