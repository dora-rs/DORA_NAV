#include <iostream>

#include "nav_msgs/odom_message.hpp"
#include "nav_msgs/odometry_with_covariance_message.hpp"
#include "nav_msgs/imu_message.hpp"
#include "nav_msgs/nav_sat_fix_message.hpp"
#include "nav_msgs/path_message.hpp"
#include "nav_msgs/point_cloud_message.hpp"
#include "nav_msgs/twist_message.hpp"

int main() {
    HeaderMessage header{{1717029202, 123456789}, "odom", 42};

    TwistMessage cmd_vel;
    cmd_vel.header = header;
    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = 0.3;

    OdomMessage odom;
    odom.header = header;
    odom.child_frame_id = "base_link";
    odom.pose.position.x = 1.0;
    odom.pose.orientation.w = 1.0;
    odom.twist.linear.x = 0.5;
    odom.twist.angular.z = 0.3;

    OdometryWithCovarianceMessage odom_cov;
    odom_cov.header = header;
    odom_cov.child_frame_id = "base_link";
    odom_cov.pose.pose = odom.pose;
    odom_cov.twist.twist = odom.twist;
    odom_cov.pose.covariance[0] = 0.01;
    odom_cov.pose.covariance[7] = 0.01;
    odom_cov.pose.covariance[35] = 0.02;

    ImuMessage imu;
    imu.header = {{1717029202, 123456789}, "imu_link", 43};
    imu.orientation.w = 1.0;
    imu.angular_velocity.z = 0.02;
    imu.linear_acceleration.x = 0.1;

    NavSatFixMessage nav_sat_fix;
    nav_sat_fix.header = {{1717029202, 123456789}, "gps_link", 44};
    nav_sat_fix.status = {NavSatStatusMessage::STATUS_FIX, NavSatStatusMessage::SERVICE_GPS};
    nav_sat_fix.latitude = 31.2304;
    nav_sat_fix.longitude = 121.4737;
    nav_sat_fix.altitude = 4.0;
    nav_sat_fix.position_covariance_type = NavSatFixMessage::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    nav_sat_fix.position_covariance[0] = 1.0;
    nav_sat_fix.position_covariance[4] = 1.0;
    nav_sat_fix.position_covariance[8] = 2.0;

    PathMessage path;
    path.header = {{1717029202, 123456789}, "map", 1};
    path.addPose(PoseStampedMessage("map", 1.0, 2.0, 0.0));

    PointCloudXYZIMessage xyzi_cloud;
    xyzi_cloud.header = {{1717029202, 123456789}, "lidar", 45};
    xyzi_cloud.points.push_back({1.0F, 2.0F, 3.0F, 10.0F});

    PointCloudXYZITRRMessage xyzitrr_cloud;
    xyzitrr_cloud.header = {{1717029202, 123456789}, "lidar", 46};
    xyzitrr_cloud.points.push_back({1.0F, 2.0F, 3.0F, 10.0F, 0.001F, 5, 0});

    nlohmann::json j;
    j["cmd_vel"] = cmd_vel;
    j["odom"] = odom;
    j["odom_cov"] = odom_cov;
    j["imu"] = imu;
    j["nav_sat_fix"] = nav_sat_fix;
    j["path"] = path;
    j["point_cloud_xyzi"] = xyzi_cloud;
    j["point_cloud_xyzitrr"] = xyzitrr_cloud;

    std::cout << j.dump(2) << std::endl;
    return 0;
}
