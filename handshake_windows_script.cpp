// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // RealSense API
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono> // For time tracking

// Define depth thresholds (in mm)
const int MIN_DEPTH = 300;  // Ignore anything closer than 0.5m
const int MAX_DEPTH = 3000; // Ignore anything farther than 4m

// Define region of interest (ROI) to ignore floor
const int ROI_TOP = 100;     // Ignore pixels above this line
const int ROI_BOTTOM = 350;  // Ignore the floor
const int ROI_LEFT = 100;    // Focus on the central region
const int ROI_RIGHT = 540;   // Ignore far-left and far-right objects

// Define plot settings
const int PLOT_WIDTH = 600;
const int PLOT_HEIGHT = 300;
const int MAX_POINTS = 100; // Number of points to keep in plot

std::vector<float> distance_history; // Stores past distances
std::vector<float> velocity_history; // Stores velocity estimates
std::chrono::steady_clock::time_point last_time; // Stores the last timestamp

// Convert depth frame to OpenCV Mat (16-bit grayscale)
cv::Mat depth_to_mat(const rs2::depth_frame& f) {
    return cv::Mat(cv::Size(f.get_width(), f.get_height()), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
}

// Function to draw a real-time plot
cv::Mat draw_plot(const std::vector<float>& data, const std::string& label, cv::Scalar color) {
    cv::Mat plot = cv::Mat::zeros(PLOT_HEIGHT, PLOT_WIDTH, CV_8UC3);

    if (data.size() < 2) return plot; // Not enough data to plot

    float max_value = *std::max_element(data.begin(), data.end());
    float min_value = *std::min_element(data.begin(), data.end());

    if (max_value == min_value) max_value += 0.1f; // Prevent divide-by-zero

    // Draw the plot line
    for (size_t i = 1; i < data.size(); i++) {
        int x1 = (i - 1) * (PLOT_WIDTH / MAX_POINTS);
        int y1 = PLOT_HEIGHT - ((data[i - 1] - min_value) / (max_value - min_value)) * PLOT_HEIGHT;
        int x2 = i * (PLOT_WIDTH / MAX_POINTS);
        int y2 = PLOT_HEIGHT - ((data[i] - min_value) / (max_value - min_value)) * PLOT_HEIGHT;

        cv::line(plot, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
    }

    // Add label
    cv::putText(plot, label, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    return plot;
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

class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int fd;
#endif
    bool connected;

public:
    SerialPort(const char* portName) {
        connected = false;

#ifdef _WIN32
        // Windows-specific code
        std::string fullPortName = "\\\\.\\";
        fullPortName += portName;

        hSerial = CreateFileA(fullPortName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Error opening serial port " << portName << std::endl;
            return;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error getting serial parameters" << std::endl;
            CloseHandle(hSerial);
            return;
        }

        dcbSerialParams.BaudRate = CBR_9600;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error setting serial parameters" << std::endl;
            CloseHandle(hSerial);
            return;
        }

        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Error setting timeouts" << std::endl;
            CloseHandle(hSerial);
            return;
        }
#else
        // Unix-based systems
        fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            std::cerr << "Error opening serial port " << portName << std::endl;
            return;
        }

        struct termios options;
        tcgetattr(fd, &options);

        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        tcsetattr(fd, TCSANOW, &options);
#endif

        connected = true;
        std::cout << "Connected to " << portName << std::endl;
    }

    ~SerialPort() {
        if (connected) {
#ifdef _WIN32
            CloseHandle(hSerial);
#else
            close(fd);
#endif
        }
    }

    bool write(const char* data, size_t size) {
        if (!connected) return false;

#ifdef _WIN32
        DWORD bytesWritten;
        return WriteFile(hSerial, data, size, &bytesWritten, NULL);
#else
        return ::write(fd, data, size) != -1;
#endif
    }
};
// END CLAUDE CODE

int main() try {

    // Replace with your actual serial port
    #ifdef _WIN32
        SerialPort serialPort("COM5");  // Windows style port
    #else
        SerialPort serialPort("/dev/ttyACM0");  // Linux/Mac style port
    #endif
    std::cout << "Sending binary values over serial. Press Ctrl+C to exit." << std::endl;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    bool doorOpen = false;
    float doorOpenThreshold = 0.7; // Distance in m(?) that door opens

    last_time = std::chrono::steady_clock::now(); // Initialize time tracking

    while (true) {
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

        // Find the minimum depth value in the ROI (runner’s position)
        double min_val;
        cv::minMaxIdx(roi_depth, &min_val, nullptr, nullptr, nullptr, roi_mask);

        // Convert mm to meters
        float runner_distance_m = min_val / 1000.0;

        // Get current time
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed_time = now - last_time;
        last_time = now; // Update last timestamp

        // Compute velocity (change in distance / time)
        float velocity_m_s = 0.0;
        if (!distance_history.empty()) {
            float last_distance = distance_history.back();
            velocity_m_s = (runner_distance_m - last_distance) / elapsed_time.count();
        }

        // Store values for plotting
        distance_history.push_back(runner_distance_m);
        velocity_history.push_back(velocity_m_s);
        if (distance_history.size() > MAX_POINTS) distance_history.erase(distance_history.begin());
        if (velocity_history.size() > MAX_POINTS) velocity_history.erase(velocity_history.begin());

        // Display distance and velocity
        // std::cout << "Runner Distance: " << runner_distance_m << " meters | Velocity: " << velocity_m_s << " m/s" << std::endl;
        
        // "Open" the door if the right distance
        /*if (runner_distance_m < doorOpenThreshold) {
            doorOpen = true;
        }*/
        bool lastDoorOpen = doorOpen;
        doorOpen = runner_distance_m < doorOpenThreshold;
        // Convert to character '0' or '1'
        if (lastDoorOpen != doorOpen) {
            // Value has changed, send update signal
            char sendValue = doorOpen ? '1' : '0';

            // Send value
            serialPort.write(&sendValue, 1);

            // Print to console
            std::cout << "Sent: " << (doorOpen ? "1" : "0") << std::endl;
        }


        // Output door distance and 
        std::cout << "Runner Distance: " << runner_distance_m;
        if (doorOpen) {
            std::cout << "Door open." << std::endl;
        }
        else {
            std::cout << "Door not open." << std::endl;
        }
        // Send information to Arduino

        
        // Debugging: Draw ROI on depth map
        cv::Mat depth_colormap;
        depth_float.convertTo(depth_colormap, CV_8UC1, 255.0 / MAX_DEPTH);
        cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);
        cv::rectangle(depth_colormap, roi, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Depth Frame (ROI Highlighted)", depth_colormap);

        // Generate and display the distance plot
        cv::Mat distance_plot = draw_plot(distance_history, "Distance (m)", cv::Scalar(0, 255, 0));

        // Generate and display the velocity plot
        cv::Mat velocity_plot = draw_plot(velocity_history, "Velocity (m/s)", cv::Scalar(255, 0, 0));

        // Show both plots
        cv::imshow("Runner Distance Plot", distance_plot);
        cv::imshow("Runner Velocity Plot", velocity_plot);
        

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
