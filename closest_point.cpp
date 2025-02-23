// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <common/cli.h>


#include <iostream>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

void plotHistogram(const cv::Mat& depth_image) {
    // Define histogram parameters
    int histSize = 512; // Number of bins
    float range[] = { 0, 65535 }; // Depth values range (16-bit max)
    const float* histRange = { range };

    cv::Mat hist;
    cv::calcHist(&depth_image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // Normalize histogram for visualization
    int hist_w = 1024, hist_h = 400;
    int bin_w = cvRound((double)hist_w / histSize);
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    // Normalize histogram
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX);

    // Draw histogram
    for (int i = 1; i < histSize; i++) {
        cv::line(histImage,
            cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
            cv::Point(bin_w * i, hist_h - cvRound(hist.at<float>(i))),
            cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("Depth Histogram", histImage);
}

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char* argv[]) try
{
    auto settings = rs2::cli("hello-realsense example")
        .process(argc, argv);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p(settings.dump());

    // Configure and start the pipeline
    p.start();

    while (true)
    {
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        const int w = depth.get_width();
        const int h = depth.get_height();

        // Convert RealSense depth frame to OpenCV Mat (16-bit grayscale)
        cv::Mat depth_image(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        // Normalize for display
        cv::Mat depth_colormap;
        double min, max;
        cv::minMaxIdx(depth_image, &min, &max);
        depth_image.convertTo(depth_colormap, CV_8UC1, 255.0 / max);
        cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

        // Show depth frame
        cv::imshow("Depth Frame", depth_colormap);

        // Plot histogram
        plotHistogram(depth_image);

        if (cv::waitKey(1) == 27) break; // Press 'ESC' to exit
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
