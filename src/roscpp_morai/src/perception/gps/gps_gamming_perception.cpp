/**
GPS Gamming Detection System
- Detects GPS signal interruptions (outages)
- Recognizes and logs gamming intervals
- Provides statistical information
*/

#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <vector>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <chrono>
#include <iomanip>
#include "hybrid_function.hpp"
using namespace std;

// ====================== GPS GAMMING STATE ======================
struct GPSGammingState {
    bool is_gamming = false;
    ros::Time last_valid_gps_time;
    ros::Time gamming_start_time;
    double gps_timeout_threshold = 2.0;  // GPS timeout threshold (seconds)
    
    // Statistics
    int gamming_count = 0;  // Number of gamming occurrences
    double total_gamming_duration = 0.0;  // Total gamming duration
    vector<pair<ros::Time, ros::Time>> gamming_segments;  // (start_time, end_time) pairs
};

GPSGammingState gps_state;
bool gps_first_received = false;

// ====================== HELPER FUNCTIONS ======================
/**
 * Detects GPS gamming state
 * - Checks GPS validity
 * - Detects gamming intervals
 * - Updates statistics
 */
void detectGPSGamming(const ros::Time& current_time) {
    if (!gps_first_received) return;
    
    double time_since_last_gps = (current_time - gps_state.last_valid_gps_time).toSec();
    
    // GPS timeout detection
    if (time_since_last_gps > gps_state.gps_timeout_threshold) {
        if (!gps_state.is_gamming) {
            // Start of new gamming interval
            gps_state.is_gamming = true;
            gps_state.gamming_start_time = gps_state.last_valid_gps_time;
            gps_state.gamming_count++;
            
            ROS_WARN("[GPS GAMMING] ===== GPS Signal Loss Detected =====");
            ROS_WARN("[GPS GAMMING] Gamming occurrence count: %d", gps_state.gamming_count);
            ROS_WARN("[GPS GAMMING] Last valid GPS time: %.3f", gps_state.gamming_start_time.toSec());
        }
    } else {
        if (gps_state.is_gamming) {
            // End of gamming interval (GPS signal recovered)
            gps_state.is_gamming = false;
            double gamming_duration = (current_time - gps_state.gamming_start_time).toSec();
            gps_state.total_gamming_duration += gamming_duration;
            gps_state.gamming_segments.push_back(
                {gps_state.gamming_start_time, current_time}
            );
            
            ROS_WARN("[GPS GAMMING] ===== GPS Signal Recovered =====");
            ROS_WARN("[GPS GAMMING] Gamming duration: %.3f sec", gamming_duration);
            ROS_WARN("[GPS GAMMING] Total gamming time: %.3f sec", gps_state.total_gamming_duration);
            ROS_WARN("[GPS GAMMING] Gamming interval: [%.3f ~ %.3f]", 
                     gps_state.gamming_start_time.toSec(), current_time.toSec());
        }
    }
}

/**
 * Prints GPS gamming statistics
 */
void printGPSGammingStatistics() {
    ROS_INFO("\n========== GPS GAMMING STATISTICS ==========");
    ROS_INFO("Total gamming occurrences: %d", gps_state.gamming_count);
    ROS_INFO("Total gamming duration: %.3f sec", gps_state.total_gamming_duration);
    ROS_INFO("Current gamming state: %s", gps_state.is_gamming ? "IN PROGRESS" : "NORMAL");
    
    if (!gps_state.gamming_segments.empty()) {
        ROS_INFO("\nGameming intervals details:");
        for (size_t i = 0; i < gps_state.gamming_segments.size(); i++) {
            double start = gps_state.gamming_segments[i].first.toSec();
            double end = gps_state.gamming_segments[i].second.toSec();
            double duration = end - start;
            ROS_INFO("  [%lu] %.3f ~ %.3f (%.3f sec)", i + 1, start, end, duration);
        }
    }
    ROS_INFO("============================================\n");
}

// ====================== CALLBACK ======================
void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    double x, y, z;
    ros::Time current_time = ros::Time::now();
    
    // Check GPS validity (lat/lon should not be 0)
    if (msg->latitude == 0.0 && msg->longitude == 0.0) {
        // No GPS signal, run gamming detection
        detectGPSGamming(current_time);
        return;
    }
    
    // Valid GPS signal received
    if (!gps_first_received) {
        gps_first_received = true;
        ROS_INFO("[GPS] First GPS signal received");
    }
    
    // Update last valid GPS time
    gps_state.last_valid_gps_time = current_time;
    gps_state.is_gamming = false;  // Clear gamming state when GPS signal is present
    
    // Coordinate transformation (WGS84 to ENU)
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               lat0, lon0, h0, x, y, z);
    ego.x = x;
    ego.y = y;
    
    // Check gamming state
    detectGPSGamming(current_time);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_gamming_perception");
    ros::NodeHandle nh;

    // Get ROS package path for file locations
    std::string pkg = ros::package::getPath("roscpp_morai");

    // Set GPS timeout threshold (unit: seconds)
    // Can be loaded from YAML or use default value
    gps_state.gps_timeout_threshold = 2.0;  // Default: 2 seconds
    
    ROS_INFO("[GPS GAMMING] Initialization complete");
    ROS_INFO("[GPS GAMMING] GPS timeout threshold: %.1f sec", gps_state.gps_timeout_threshold);

    ros::Subscriber gps_sub = nh.subscribe("/gps", 10, gpsCallback);
    
    // Periodic statistics output (every 3 seconds)
    ros::Timer stat_timer = nh.createTimer(ros::Duration(3.0), 
        [](const ros::TimerEvent&) {
            printGPSGammingStatistics();
        }
    );

    ros::spin();
    
    // Final statistics output on shutdown
    ROS_INFO("\n[GPS GAMMING] Program terminating");
    printGPSGammingStatistics();
    
    return 0;
}