//
// Created by Hasan on 11/13/26.
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <boost/filesystem.hpp>

#include "example-imgui.hpp"


#include "imgui_impl_glfw.h"
#include <imgui_impl_opengl3.h>
#include <realsense_imgui.h>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sstream>

/*
 This example introduces the concept of spatial stream alignment.
 For example usecase of alignment, please check out align-advanced and measure demos.
 The need for spatial alignment (from here "align") arises from the fact
 that not all camera streams are captured from a single viewport.
 Align process lets the user translate images from one viewport to another.
 That said, results of align are synthetic streams, and suffer from several artifacts:
 1. Sampling - mapping stream to a different viewport will modify the resolution of the frame
               to match the resolution of target viewport. This will either cause downsampling or
               upsampling via interpolation. The interpolation used needs to be of type
               Nearest Neighbor to avoid introducing non-existing values.
 2. Occlussion - Some pixels in the resulting image correspond to 3D coordinates that the original
               sensor did not see, because these 3D points were occluded in the original viewport.
               Such pixels may hold invalid texture values.
*/

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
    to_depth,
    to_color
};

// Forward definition of UI rendering, implemented below
void render_slider(rect location, float* alpha, direction* dir);
int get_video_dim(const rs2::pipeline_profile& profile, int& width, int& height);
int write_distance_info(const std::string& filename,  float* values, int size);

namespace fs = boost::filesystem;

int main(int argc, char * argv[]) try
{
    // auto settings = rs2::cli( "rs-align example" )
    //     .process( argc, argv );

    // rs2::context ctx( settings.dump() );
    // std::string serial;
    // if( ! device_with_streams( ctx, { RS2_STREAM_COLOR, RS2_STREAM_DEPTH }, serial ) )
    //     return EXIT_SUCCESS;

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0]
             << " <output_directory>" << std::endl;
        return -1;
    }

    std::string imageDir = (fs::path(argv[1]) / "frames").string();
    std::string depthInfoDir = (fs::path(argv[1]) / "depth_info").string();

    mkdir(imageDir.c_str(), 0755);
    mkdir(depthInfoDir.c_str(), 0755);



    // Create and initialize GUI related objects
    window app(1280, 720, "RealSense Align Example"); // Simple window handling
    // Setup Dear ImGui context
    ImGui::CreateContext();
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(app, false);
    ImGui_ImplOpenGL3_Init();
    rs2::colorizer c;                     // Helper to colorize depth images
    texture depth_image, color_image;     // Helpers for renderig images

    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    rs2::config cfg;
    // if (!serial.empty())
    //     cfg. (serial);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR
        // ,640, 480
        //  ,RS2_FORMAT_RGB8
        //  ,30
        );
    auto profile = pipe.start(cfg);
    int width, height;
    if (get_video_dim(profile, width, height) != 0) {
        std::cerr << "Failed to get vidceo dim " << std::endl;
        return -1;
    }

    std::vector<float> depth_arr(width * height);





    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    float       alpha = 0.5f;               // Transparancy coefficient
    direction   dir = direction::to_color;  // Alignment direction


    int count = 0;
    int frameIndex = 0;
    while (app) // Application still alive?
    {
        // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        if (dir == direction::to_depth)
        {
            // Align all frames to depth viewport
            frameset = align_to_depth.process(frameset);
        }
        else
        {
            // Align all frames to color viewport
            frameset = align_to_color.process(frameset);
        }

        // With the aligned frameset we proceed as usual
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        auto colorized_depth = c.colorize(depth);

        glEnable(GL_BLEND);
        // Use the Alpha channel for blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (dir == direction::to_depth)
        {
            // When aligning to depth, first render depth image
            // and then overlay color on top with transparancy
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });
            color_image.render(color, { 0, 0, app.width(), app.height() }, alpha);
            // std::cout << "dim: " << color.get_width() << " X " << color.get_height() << std::endl;
            // std::cout << "dim: " << depth.get_width() << " X " << depth.get_height() << std::endl;

        }
        else
        {
            // When aligning to color, first render color image
            // and then overlay depth image on top
            color_image.render(color, { 0, 0, app.width(), app.height() });
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() }, 1 - alpha);
            if (count >= 50) {
                cv::Mat image(
                    color.get_height(),
                    color.get_width(),
                    CV_8UC3,
                    (void*)color.get_data(),
                    cv::Mat::AUTO_STEP
                );

                cv::Mat grayFrame;
                cv::cvtColor(image, grayFrame, CV_RGB2GRAY);

                // Generate output filename
                std::stringstream image_filename;
                image_filename << imageDir << "/frame_"
                         << std::setfill('0') << std::setw(6)
                         << frameIndex << ".pgm";

                // Save grayscale frame as PGM
                if (!imwrite(image_filename.str(), grayFrame))
                {
                    std::cerr << "Error saving: "
                         << image_filename.str() << std::endl;
                    break;
                }




                float dist_to_center = depth.get_distance(width / 2, height / 2);
                std::cout << "width: " << width << std::endl;
                assert(width == depth.get_width());
                assert(height == depth.get_height());
                int index = 0;
                for (int i=0; i < height; ++i ) {
                    for (int j=0; j < width; ++j) {
                        depth_arr[index++] = depth.get_distance(j, i);
                    }


                }

                std::stringstream depth_info_filename;
                depth_info_filename << depthInfoDir << "/depth_"
                         << std::setfill('0') << std::setw(6)
                         << frameIndex << ".dp";

                if (write_distance_info(depth_info_filename.str().c_str(), &depth_arr[0], depth_arr.size()) != 0) {
                    std::cerr << "Error writing depth info to " << depth_info_filename.str() << std::endl;
                }

                frameIndex++;


            }
            // std::cout << "dim: " << color.get_width() << " X " << color.get_height() << std::endl;
            // std::cout << "dim: " << depth.get_width() << " X " << depth.get_height() << std::endl;
        }

        glColor4f(1.f, 1.f, 1.f, 1.f);
        glDisable(GL_BLEND);

        count ++;
        // Render the UI:
        RsImGui::PushNewFrame();
        render_slider({ 15.f, app.height() - 60, app.width() - 30, app.height() }, &alpha, &dir);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    }
    RsImGui::PopNewFrame();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void render_slider(rect location, float* alpha, direction* dir)
{
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;

    ImGui::SetNextWindowPos({ location.x, location.y });
    ImGui::SetNextWindowSize({ location.w, location.h });
    ImGuiSetStyleColors();

    // Render transparency slider:
    ImGui::Begin("slider", nullptr, flags);
    ImGui::PushItemWidth(-1);
    ImGui::SliderFloat("##Slider", alpha, 0.f, 1.f);
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        RsImGui::CustomTooltip("Texture Transparancy: %.3f", *alpha);

    // Render direction checkboxes:
    bool to_depth = (*dir == direction::to_depth);
    bool to_color = (*dir == direction::to_color);

    if (ImGui::Checkbox("Align To Depth", &to_depth))
    {
        *dir = to_depth ? direction::to_depth : direction::to_color;
    }
    ImGui::SameLine();
    ImGui::SetCursorPosX(location.w - 140);
    if (ImGui::Checkbox("Align To Color", &to_color))
    {
        *dir = to_color ? direction::to_color : direction::to_depth;
    }

    ImGui::End();
}

int get_video_dim(const rs2::pipeline_profile& profile, int& width, int& height) {

    // Get active stream profiles
    for (auto&& stream : profile.get_streams())
    {
        auto video = stream.as<rs2::video_stream_profile>();

        if (video)
        {
            std::cout << "Stream: " << stream.stream_name() << std::endl;
            std::cout << "Width: " << video.width() << std::endl;
            std::cout << "Height: " << video.height() << std::endl;
            std::cout << "FPS: " << video.fps() << std::endl;
            std::cout << "Format: " << video.format() << std::endl;
            width = video.width();
            height = video.height();
            return 0;
        }

    }
    return -1;

}

int write_distance_info(const std::string& filename,  float* values, int size) {
    std::ofstream file(filename);

    if (!file.is_open())
        return -1;

    for (int i = 0; i < size; ++i)
    {
        file << values[i] << "\n";
    }

    file.close();

    return 0;
}