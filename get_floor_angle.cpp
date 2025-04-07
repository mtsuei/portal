/*
    This is the main code to operate Portal from the PC side. It has two main functions:
    1) Interface with the camera and proccess its results to get position and velocity estimations
    2) Communicate with the handshake for diagnostic and control.

    TODO: redo velocity estimation (moving average instead of position?)
    TODO: incorporate camera status in signal to arduino
*/

#include <librealsense2/rs.hpp> // RealSense API
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono> // For time tracking

// Define depth thresholds (in mm)
const int MIN_DEPTH = 50;  // Ignore anything closer than 0.5m
const int MAX_DEPTH = 3000; // Ignore anything farther than 4m

// Define region of interest (ROI) to ignore floor
//const int ROI_TOP = 100;     // Ignore pixels above this line
//const int ROI_BOTTOM = 350;  // Ignore the floor
//const int ROI_LEFT = 100;    // Focus on the central region
//const int ROI_RIGHT = 540;   // Ignore far-left and far-right objects

// Smaller ROI definition for bench test
const int ROI_TOP = 350;     // Ignore pixels above this line
const int ROI_BOTTOM = 450;  // Ignore the floor
const int ROI_LEFT = 250;    // Focus on the central region
const int ROI_RIGHT = 350;   // Ignore far-left and far-right objects

// Define plot settings
const int PLOT_WIDTH = 600;
const int PLOT_HEIGHT = 300;
const int MAX_POINTS = 100; // Number of points to keep in plot

// Convert depth frame to OpenCV Mat (16-bit grayscale)
cv::Mat depth_to_mat(const rs2::depth_frame& f) {
    return cv::Mat(cv::Size(f.get_width(), f.get_height()), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
}

// BEGIN CLAUDE CODE
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

// BEGIN CHAT GPT GENERATED MOVING AVERAGE CLASS
#include <queue>

class MovingAverage {
    int size;
    std::queue<int> window;
    double sum;

public:
    MovingAverage(int windowSize) : size(windowSize), sum(0) {}

    double next(int val) {
        if (window.size() == size) {
            sum -= window.front();
            window.pop();
        }
        window.push(val);
        sum += val;
        return sum / window.size();
    }

    double getAverage() const {
        return window.empty() ? 0 : sum / window.size();
    }
};
// END MOVING AVERAGE CLASS

int main() try {
    // Initialize realsense data pipelines
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    // Initialize door state variables
    bool isDoorOpen = false;
    MovingAverage averageAngle(50);

    while (true) {
        /* ------------------------- CAMERA & IMAGE PROCCESSING ------------------------- */
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        // Convert RealSense depth frame to OpenCV Mat
        cv::Mat depth_mat = depth_to_mat(depth);

        // Convert depth to float for processing
        cv::Mat depth_float;
        depth_mat.convertTo(depth_float, CV_32F);

        // Mask out invalid depth values
        cv::Mat valid_depth_mask = (depth_float > MIN_DEPTH) & (depth_float < MAX_DEPTH);
        depth_float.setTo(MAX_DEPTH, ~valid_depth_mask);

        // Extract ROI (Region of Interest)
        cv::Rect roi(ROI_LEFT, ROI_TOP, ROI_RIGHT - ROI_LEFT, ROI_BOTTOM - ROI_TOP);
        cv::Mat roi_depth = depth_float(roi);
        cv::Mat roi_mask = valid_depth_mask(roi);

        // Add angle code in here
        int roi_center_col = roi.width / 2;

        // Define narrow strips at the top and bottom of the ROI
        cv::Rect top_strip_rect(roi.x + roi_center_col - 5, roi.y, 10, 10);
        cv::Rect bottom_strip_rect(roi.x + roi_center_col - 5, roi.y + roi.height - 10, 10, 10);

        // Extract strips from the full depth matrix
        cv::Mat top_strip = depth_float(top_strip_rect);
        cv::Mat bottom_strip = depth_float(bottom_strip_rect);
        cv::Mat top_mask = valid_depth_mask(top_strip_rect);
        cv::Mat bottom_mask = valid_depth_mask(bottom_strip_rect);

        // Compute average depth in each strip
        double top_avg = cv::mean(top_strip, top_mask)[0];
        double bottom_avg = cv::mean(bottom_strip, bottom_mask)[0];

        // Get camera intrinsics
        rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        // Estimate declination angle
        double depth_diff = top_avg - bottom_avg;
        double vertical_distance = (double)(bottom_strip_rect.y - top_strip_rect.y);
        double pixel_to_angle = atan2(vertical_distance, top_avg); // approximate

        double angle_deg = atan2(depth_diff, vertical_distance * top_avg / intr.fy) * 180.0 / CV_PI;
        averageAngle.next(angle_deg);
        std::cout << "Estimated declination angle: " << angle_deg << " degrees" << std::endl
            << " | 25 cycle moving average: " << averageAngle.getAverage() << " degrees";


        // Debugging: Draw ROI on depth map
        cv::Mat depth_colormap;
        depth_float.convertTo(depth_colormap, CV_8UC1, 255.0 / MAX_DEPTH);
        cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);
        cv::rectangle(depth_colormap, roi, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Depth Frame (ROI Highlighted)", depth_colormap);

        if (cv::waitKey(1) == 27) break; // Press 'ESC' to exit
    }

    return 0;
}
catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function()
        << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
