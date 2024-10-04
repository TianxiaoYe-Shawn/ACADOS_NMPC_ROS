#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Function to generate a circle trajectory
nav_msgs::Path generate_circle_trajectory(double radius, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    for (int i = 0; i < points; i++) {
        double angle = i * 6 * M_PI / points;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;
        pose.pose.position.x = radius * cos(angle);
        pose.pose.position.y = radius * sin(angle);
        pose.pose.position.z = 0;  // Assuming flat ground
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a figure-eight trajectory
nav_msgs::Path generate_figure_eight_trajectory(double radius, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    for (int i = 0; i < points; i++) {
        double t = (double)i / points * 6 * M_PI; // parameter t goes from 0 to 2*PI
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Figure-eight parametric equations
        pose.pose.position.x = radius * sin(t);
        pose.pose.position.y = radius * sin(t) * cos(t);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_t = (double)(i+1) / points * 2 * M_PI;
        double next_x = radius * sin(next_t);
        double next_y = radius * sin(next_t) * cos(next_t);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a wave trajectory
nav_msgs::Path generate_wave_trajectory(double amplitude, double wavelength, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    // Wave along the x-axis
    for (int i = 0; i < points; i++) {
        double x = (double)i / points * wavelength;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Wave parametric equations
        pose.pose.position.x = x;
        pose.pose.position.y = amplitude * sin(2 * M_PI * x / wavelength);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_x = (double)(i + 1) / points * wavelength;
        double next_y = amplitude * sin(2 * M_PI * next_x / wavelength);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a multi-wave trajectory
nav_msgs::Path generate_multi_wave_trajectory(double amplitude, double wavelength, int waves, int points_per_wave, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    // Total points are points per wave multiplied by number of waves
    int total_points = points_per_wave * waves;
    
    // Extend the wave across multiple wavelengths
    for (int i = 0; i < total_points; i++) {
        double x = (double)i / points_per_wave * wavelength;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Multi-wave parametric equations
        pose.pose.position.x = x;
        pose.pose.position.y = amplitude * sin(2 * M_PI * x / wavelength);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_x = (double)(i + 1) / points_per_wave * wavelength;
        double next_y = amplitude * sin(2 * M_PI * next_x / wavelength);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a rounded rectangle trajectory
nav_msgs::Path generate_rounded_rectangle_trajectory(double width, double height, double corner_radius, int points_per_corner, int points_per_side, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    // Ensure the corner radius is not too large
    corner_radius = std::min(corner_radius, std::min(width / 2, height / 2));

    // Define the four corners of the rectangle (these are the centers of the rounded corners)
    geometry_msgs::PoseStamped bottom_left, bottom_right, top_right, top_left;
    bottom_left.pose.position.x = -width / 2 + corner_radius; 
    bottom_left.pose.position.y = -height / 2 + corner_radius;

    bottom_right.pose.position.x = width / 2 - corner_radius;
    bottom_right.pose.position.y = -height / 2 + corner_radius;

    top_right.pose.position.x = width / 2 - corner_radius;
    top_right.pose.position.y = height / 2 - corner_radius;

    top_left.pose.position.x = -width / 2 + corner_radius;
    top_left.pose.position.y = height / 2 - corner_radius;

    // Helper function to generate a quarter-circle for rounded corners
    auto generate_corner = [&](double center_x, double center_y, double start_angle, double end_angle) {
        for (int i = 0; i <= points_per_corner; i++) {
            double angle = start_angle + i * (end_angle - start_angle) / points_per_corner;
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = frame_id;

            pose.pose.position.x = center_x + corner_radius * cos(angle);
            pose.pose.position.y = center_y + corner_radius * sin(angle);
            pose.pose.position.z = 0;

            // Use tf2 for quaternion calculation
            tf2::Quaternion quat;
            quat.setRPY(0, 0, angle);
            pose.pose.orientation = tf2::toMsg(quat);

            path.poses.push_back(pose);
        }
    };

    // Generate the path for the rounded rectangle
    // 1. Bottom-right corner (from -π/2 to 0)
    generate_corner(bottom_right.pose.position.x, bottom_right.pose.position.y, -M_PI / 2, 0);

    // 2. Right side (vertical line between bottom-right and top-right)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        pose.pose.position.x = width / 2;
        pose.pose.position.y = -height / 2 + corner_radius + i * (height - 2 * corner_radius) / points_per_side;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);  // Facing upwards

        path.poses.push_back(pose);
    }

    // 3. Top-right corner (from 0 to π/2)
    generate_corner(top_right.pose.position.x, top_right.pose.position.y, 0, M_PI / 2);

    // 4. Top side (horizontal line between top-right and top-left)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        pose.pose.position.x = width / 2 - corner_radius - i * (width - 2 * corner_radius) / points_per_side;
        pose.pose.position.y = height / 2;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);  // Facing left

        path.poses.push_back(pose);
    }

    // 5. Top-left corner (from π/2 to π)
    generate_corner(top_left.pose.position.x, top_left.pose.position.y, M_PI / 2, M_PI);

    // 6. Left side (vertical line between top-left and bottom-left)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        pose.pose.position.x = -width / 2;
        pose.pose.position.y = height / 2 - corner_radius - i * (height - 2 * corner_radius) / points_per_side;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);  // Facing downwards

        path.poses.push_back(pose);
    }

    // 7. Bottom-left corner (from π to 3π/2)
    generate_corner(bottom_left.pose.position.x, bottom_left.pose.position.y, M_PI, 3 * M_PI / 2);

    // 8. Bottom side (horizontal line between bottom-left and bottom-right)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        pose.pose.position.x = -width / 2 + corner_radius + i * (width - 2 * corner_radius) / points_per_side;
        pose.pose.position.y = -height / 2;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);  // Facing right

        path.poses.push_back(pose);
    }

    tf2::Quaternion quat;
    tf2::fromMsg(path.poses[0].pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM("First point in path: x = " << path.poses[0].pose.position.x 
                    << ", y = " << path.poses[0].pose.position.y 
                    << ", yaw = " << yaw);
                
    return path;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("/trajectory", 10, true);

    // Parameters for the circle trajectory
    double radius = 10.0;  // Radius of the circle
    double amplitude = 1.0;  // Amplitude of the waves
    double wavelength = 10.0; // Wavelength of the waves
    int waves = 5;           // Number of waves
    int points_per_wave = 300;  // Number of points in each wave
    int points = 5000;     // Number of points in the circle
    // Parameters for the rounded rectangle trajectory
    double width = 90.0;        // Width of the rectangle
    double height = 90.0;        // Height of the rectangle
    double corner_radius = 3.0; // Radius of the rounded corners
    int points_per_corner = 100; // Number of points for each corner
    int points_per_side = 1000;  // Number of points for each side
    std::string frame_id = "map";  // Or the frame you want to use

    // Generate the trajectory
    //nav_msgs::Path path = generate_circle_trajectory(radius, points, frame_id);
    nav_msgs::Path path = generate_figure_eight_trajectory(radius, points, frame_id);
    //nav_msgs::Path path = generate_rounded_rectangle_trajectory(width, height, corner_radius, points_per_corner, points_per_side, frame_id);
    //nav_msgs::Path path = generate_wave_trajectory(amplitude, wavelength, points, frame_id);
    //nav_msgs::Path path = generate_multi_wave_trajectory(amplitude, wavelength, waves, points_per_wave, frame_id);
    
    // Publish the trajectory
    trajectory_pub.publish(path);

    ros::spin();

    return 0;
}
