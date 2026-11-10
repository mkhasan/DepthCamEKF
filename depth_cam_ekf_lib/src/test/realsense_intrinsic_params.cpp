//
// Created by Hasan on 11/9/26.
//


#include <iostream>
#include <librealsense2/rs.hpp>

using namespace std;

int main() {

    rs2::pipeline pipe;
    rs2::pipeline_profile profile = pipe.start();

    auto color = profile.get_stream(RS2_STREAM_COLOR)
                      .as<rs2::video_stream_profile>();

    rs2_intrinsics intr = color.get_intrinsics();

    std::cout << "Distortion model: "
              << intr.model << std::endl;

    std::cout << "Coefficients: ";
    for (float c : intr.coeffs)
        std::cout << c << " ";
    std::cout << std::endl;
        return 0;


}