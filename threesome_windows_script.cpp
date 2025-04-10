/*
    This is the main code to operate Portal from the PC side. It has two main functions:
    1) Interface with the camera and proccess its results to get position and velocity estimations
    2) Communicate with the handshake for diagnostic and control.

    TODO: redo velocity estimation (moving average instead of position?)
    TODO: incorporate camera status in signal to arduino
    TODO: make sure we're not querying camera RGB output
*/

#include <librealsense2/rs.hpp> // RealSense API
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono> // For time tracking
#include <thread>
#include <cstdlib>

#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

/* --------------------- DEFINE ARDUINO COMMS FUNCTIONS --------------------- */
// Written with Claude
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
/* -------------------------------------------------------------------------- */

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

// Define region of interest (ROI) to ignore floor
const int ROI_TOP = 50;     // Ignore pixels above this line
const int ROI_BOTTOM = 350;  // Ignore the floor
const int ROI_LEFT = 120;    // Focus on the central region
const int ROI_RIGHT = 520;   // Ignore far-left and far-right objects

// Smaller ROI definition for bench test
//const int ROI_TOP = 150;     // Ignore pixels above this line
//const int ROI_BOTTOM = 250;  // Ignore the floor
//const int ROI_LEFT = 250;    // Focus on the central region
//const int ROI_RIGHT = 350;   // Ignore far-left and far-right objects

// Define plot settings
const int PLOT_WIDTH = 600;
const int PLOT_HEIGHT = 300;
const int MAX_POINTS = 100; // Number of points to keep in plot

std::vector<float> distance_history; // Stores past distances
std::vector<float> velocity_history; // Stores velocity estimates
std::chrono::steady_clock::time_point last_time; // Stores the last timestamp

// Function to convert depth frame to OpenCV Mat (16-bit grayscale)
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

const float cameraHeight = 2.413f;   // meters
const float cameraDecAngle = 17.0f; // degrees

const float resolution = 0.01f; // 1cm per pixel
const int map_size = 500;       // 5m x 5m map

const bool showCameraFeedROI = true;

int main() try {
    //  Declare the serial port
#ifdef _WIN32
    SerialPort serialPort("COM7");  // Windows style port
#else
    SerialPort serialPort("/dev/ttyACM0");  // Linux/Mac style port
#endif
    std::cout << "Sending binary values over serial. Press Ctrl+C to exit." << std::endl;

    rs2::pipeline pipe;
    pipe.start();

    float theta = cameraDecAngle * M_PI / 180.0f;
    float cosT = cos(theta), sinT = sin(theta);

    // Buffers
    cv::Mat zmap(map_size, map_size, CV_32F, cv::Scalar(0));
    cv::Mat count_z(map_size, map_size, CV_32F, cv::Scalar(0));

    cv::Mat xzmap(map_size, map_size, CV_32F, cv::Scalar(0));
    cv::Mat count_xz(map_size, map_size, CV_32F, cv::Scalar(0));

    float closest_point[3]; // [x, y, z]

    while (cv::waitKey(1) != 27) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        zmap.setTo(0);
        count_z.setTo(0);
        xzmap.setTo(0);
        count_xz.setTo(0);

        int w = depth.get_width(), h = depth.get_height();
        // Reset min closest point
        closest_point[1] = std::numeric_limits<float>::max();

        for (int y = ROI_TOP; y < ROI_BOTTOM; y += 2) {
            for (int x = ROI_LEFT; x < ROI_RIGHT; x += 2) {
                float dist = depth.get_distance(x, y);
                if (dist <= 0 || dist > 5.0f) continue;

                float pixel[2] = { static_cast<float>(x), static_cast<float>(y) };
                float p_cam[3];
                rs2_deproject_pixel_to_point(p_cam, &intr, pixel, dist);

                // Transform to world frame
                float Xw = p_cam[0];
                float Yw = -(p_cam[1] * cosT - p_cam[2] * sinT);
                float Zw = -p_cam[1] * sinT - p_cam[2] * cosT + cameraHeight;

                // Track closest point
                if (Yw < closest_point[1] && Zw > 0.6) {
                    closest_point[0] = Xw;
                    closest_point[1] = Yw;
                    closest_point[2] = Zw;
                }

                // Top-down view: X-Y -> color Z
                int ix = static_cast<int>(map_size / 2 + Xw / resolution);
                int iy = static_cast<int>(map_size / 2 + Yw / resolution);
                if (ix >= 0 && ix < map_size && iy >= 0 && iy < map_size) {
                    zmap.at<float>(iy, ix) += Zw;
                    count_z.at<float>(iy, ix) += 1.0f;
                }

                // Side view: X-Z -> color Y
                int iz = static_cast<int>(map_size - 1 - Zw / resolution); // Z up → image down
                if (ix >= 0 && ix < map_size && iz >= 0 && iz < map_size) {
                    xzmap.at<float>(iz, ix) += Yw;
                    count_xz.at<float>(iz, ix) += 1.0f;
                }
            }
        }

        /* --------------------- PROCESS POSITION DATA --------------------- */
        // Get current time
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed_time = now - last_time;
        last_time = now; // Update last timestamp

        // Compute velocity (change in distance / time)
        float velocity_m_s = 0.0;
        if (!distance_history.empty()) {
            float last_distance = distance_history.back();
            velocity_m_s = (closest_point[1] - last_distance) / elapsed_time.count();
        }

        // Update velocity moving average
        /*velocity_ma.next(velocity_m_s);*/

        // Store values for plotting
        distance_history.push_back(closest_point[1]);
        velocity_history.push_back(velocity_m_s);
        if (distance_history.size() > MAX_POINTS) distance_history.erase(distance_history.begin());
        if (velocity_history.size() > MAX_POINTS) velocity_history.erase(velocity_history.begin());

        // Generate and display the distance plot
        cv::Mat distance_plot = draw_plot(distance_history, "Distance (m)", cv::Scalar(0, 255, 0));

        // Generate and display the velocity plot
        cv::Mat velocity_plot = draw_plot(velocity_history, "Velocity (m/s)", cv::Scalar(255, 0, 0));

        // Show both plots
        cv::imshow("Runner Distance Plot", distance_plot);
        cv::imshow("Runner Velocity Plot", velocity_plot);

        /* --------------------- ARDUINO COMMUNICATION ---------------------- */

        // Value has changed, send update signal
        // Convert to character '0' or '1'
        char shouldDoorBeOpen = closest_point[1] < 0.2 ? '1' : '0';

        // Send value
        serialPort.write(&shouldDoorBeOpen, 1);

        // Print to console
        std::cout << "Sent: " << shouldDoorBeOpen << " | ";





        /* --------------- PLOTTING AND DEVELOPMENT OUTPUTTING -------------- */
        // After the loop
        std::cout << "Closest point: ["
            << closest_point[0] << ", "
            << closest_point[1] << ", "
            << closest_point[2] << "] at distance: "
            << closest_point[1] << " meters\n";

        // Top-down
        cv::Mat avg_z;
        cv::divide(zmap, count_z, avg_z);
        avg_z.setTo(0, count_z == 0);

        double minZ, maxZ;
        cv::minMaxLoc(avg_z, &minZ, &maxZ);
        cv::Mat z8;
        avg_z.convertTo(z8, CV_8U, 255.0 / (maxZ - minZ), -minZ * 255.0 / (maxZ - minZ));
        cv::Mat heatmap;
        cv::applyColorMap(z8, heatmap, cv::COLORMAP_JET);
        cv::imshow("Top-Down Height Map (XY)", heatmap);

        // Side view
        cv::Mat avg_y;
        cv::divide(xzmap, count_xz, avg_y);
        avg_y.setTo(0, count_xz == 0);

        double minY, maxY;
        cv::minMaxLoc(avg_y, &minY, &maxY);
        cv::Mat y8;
        avg_y.convertTo(y8, CV_8U, 255.0 / (maxY - minY), -minY * 255.0 / (maxY - minY));
        cv::Mat xzheat;
        cv::applyColorMap(y8, xzheat, cv::COLORMAP_JET);
        //cv::flip(xzheat, xzheat, 0); // 0 = flip vertically
        cv::imshow("Front View (XZ Projection)", xzheat);


        if (showCameraFeedROI) {
            cv::Mat raw_frame(depth.get_height(), depth.get_width(), CV_16U, (void*)depth.get_data());
            cv::Mat normalized;
            raw_frame.convertTo(normalized, CV_8U, 255.0 / 10000.0);
            cv::applyColorMap(normalized, normalized, cv::COLORMAP_JET);
            cv::rectangle(
                normalized,
                cv::Point(ROI_LEFT, ROI_TOP),
                cv::Point(ROI_RIGHT, ROI_BOTTOM),
                cv::Scalar(0, 255, 255),
                2
            );
            cv::imshow("Depth with ROI", normalized);
        }
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
