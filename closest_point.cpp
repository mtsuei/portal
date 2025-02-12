// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <iostream>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <common/cli.h>

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    auto settings = rs2::cli( "hello-realsense example" )
        .process( argc, argv );

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p( settings.dump() );

    // Configure and start the pipeline
    p.start();

    double N_repetitions = 20;
    while (true)
    {
        // Initialize clock
        //auto start = std::chrono::high_resolution_clock::now();

        //// Calculating cycle time
        //for (int i = 0; i < N_repetitions; i++) {

            // Block program until frames arrive
            rs2::frameset frames = p.wait_for_frames();

            // Try to get a frame of a depth image
            rs2::depth_frame depth = frames.get_depth_frame();

            // Query the distance from the camera to the object in the center of the image
            // float dist_to_center = depth.get_distance(width / 2, height / 2);

            // Get height and width
            auto width = depth.get_width();
            auto height = depth.get_height();
            
            // Convert depth frame to OpenCV format
            //cv::Mat depth_mat(cv::Size(width, height), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

            // Find closest point
            float closest_dist = depth.get_distance(0, 0);
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    float dist = depth.get_distance(i, j);
                    // std::cout << "(" << i << "," << j << ")" << "\n";
                    if (dist < closest_dist) {
                        closest_dist = dist;
                    }
                }
            }

        // Print the distance
        std::cout << "The camera is facing an object " << closest_dist* 39.3701 << " inches away \r";
        //}

        // End timer
        //auto end = std::chrono::high_resolution_clock::now();

        // Calculate duration
        // std::chrono::duration<double> duration = end - start;
        // double freq = N_repetitions / duration.count();

        // Print the frequency
        //std::cout << "The camera is operating at a frequency of" << freq << " Hz\r";
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
