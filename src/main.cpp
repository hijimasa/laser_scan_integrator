//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/create_timer_ros.h>

#include <cmath>

#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>


struct LaserPoint
{
    float direction_;
    float distance_;
};

struct LaserPointLess
{
  bool operator()(const LaserPoint& a, const LaserPoint& b) const noexcept {
    return a.direction_ < b.direction_;
  }
};

class scanMerger : public rclcpp::Node
{
    public:
    scanMerger()
    : Node("ros2_laser_scan_merger")
    {
        tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
        
        initialize_params();
        refresh_params();
        
        
        auto default_qos = rclcpp::SensorDataQoS(); //rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic2_, default_qos, std::bind(&scanMerger::scan_callback2 , this, std::placeholders::_1));
        
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(integratedTopic_, rclcpp::SystemDefaultsQoS());
        
        tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    }
    private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser1_ = _msg;
        if (laser2_)
        {
          update_point_cloud_rgb();
        }
    }
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser2_ = _msg;
    }
    
    void update_point_cloud_rgb(){
        refresh_params();
        trans1_ = tf2_->lookupTransform(integratedFrameId_, laser1_->header.frame_id, rclcpp::Time(0));
        trans2_ = tf2_->lookupTransform(integratedFrameId_, laser2_->header.frame_id, rclcpp::Time(0));
	double sensor1_r, sensor1_p, sensor1_y, sensor2_r, sensor2_p, sensor2_y;
	geometry_quat_to_rpy(&sensor1_r, &sensor1_p, &sensor1_y, trans1_.transform.rotation);
	geometry_quat_to_rpy(&sensor2_r, &sensor2_p, &sensor2_y, trans2_.transform.rotation);
	sensor1_y += laser1Alpha_;
	sensor2_y += laser2Alpha_;

        std::vector<std::array<float,2>> scan_data;
        int count = 0;
        float min_theta = 0;
        float max_theta = 0;
        if(show1_){
        //if(false){
            for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment){
		std::array<float, 2> pt;
                
                float laser_angle;
                if (fabs(sensor1_r) < M_PI/2)
                {
                    laser_angle = i;
                } 
                else
                {
                    laser_angle = -i;
                }
                float temp_x = laser1_->ranges[count] * std::cos(laser_angle) ;
                float temp_y = laser1_->ranges[count] * std::sin(laser_angle) ;
                pt[0] = temp_x * std::cos(sensor1_y) - temp_y * std::sin(sensor1_y); 
                pt[0] += trans1_.transform.translation.x + laser1XOff_;
                pt[1] = temp_x * std::sin(sensor1_y) + temp_y * std::cos(sensor1_y);
                pt[1] += trans1_.transform.translation.y + laser1YOff_;
                count++;
                
                if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_)
                {
                    continue;
                }

                float r_ = GET_R(pt[0], pt[1]);
                float theta_ = GET_THETA(pt[0], pt[1]);
                std::array<float,2> res_;
                res_[1] = r_;
                res_[0] = theta_;
                scan_data.push_back(res_);
                if(theta_ < min_theta){
                    min_theta = theta_;
                }
                if(theta_ > max_theta){
                    max_theta = theta_;
                }
            }
        }
        
        count = 0;
        if(show2_){
            for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment){
		std::array<float,2> pt;
                
                float laser_angle;
                if (fabs(sensor2_r) < M_PI/2)
                {
                    laser_angle = i;
                } 
                else
                {
                    laser_angle = -i;
                }
                float temp_x = laser2_->ranges[count] * std::cos(laser_angle) ;
                float temp_y = laser2_->ranges[count] * std::sin(laser_angle) ;
                pt[0] = temp_x * std::cos(sensor2_y) - temp_y * std::sin(sensor2_y);
                pt[0] += trans2_.transform.translation.x + laser2XOff_;
                pt[1] = temp_x * std::sin(sensor2_y) + temp_y * std::cos(sensor2_y);
                pt[1] += trans2_.transform.translation.y + laser2YOff_;
                count++;

                if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_)
                {
                    continue;
                }

                float r_ = GET_R(pt[0], pt[1]);
                float theta_ = GET_THETA(pt[0], pt[1]);
                std::array<float,2> res_;
                res_[1] = r_;
                res_[0] = theta_;
                scan_data.push_back(res_);
                if(theta_ < min_theta){
                    min_theta = theta_;
                }
                if(theta_ > max_theta){
                    max_theta = theta_;
                }
            }
        }
       
	std::sort(scan_data.begin(), scan_data.end(), [](std::array<float,2> a, std::array<float,2> b) {
            return a[0] < b[0];
        });	

        auto integrated_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        integrated_msg_->header.frame_id = integratedFrameId_;
        integrated_msg_->header.stamp = laser1_->header.stamp;
	integrated_msg_->angle_min = min_theta;
	integrated_msg_->angle_max = max_theta;
	integrated_msg_->angle_increment = laser1_->angle_increment;
	integrated_msg_->time_increment = laser1_->time_increment;
	integrated_msg_->scan_time = laser1_->scan_time;
	integrated_msg_->range_min = laser1_->range_min;
	integrated_msg_->range_max = laser1_->range_max;
	size_t i = 1;
	std::vector<float>temp_range;
	for (float angle = min_theta; angle < max_theta; angle += laser1_->angle_increment)
	{
	    while (scan_data[i][0] < angle)
	    {
	        i++;
	    }
	    if (fabs(scan_data[i][1] - scan_data[i-1][1]) < 0.2f
	        && (fabs(scan_data[i][0] - angle) < laser1_->angle_increment 
	        || fabs(scan_data[i-1][0] - angle) < laser1_->angle_increment)
	       )
	    {
                float range = interpolate(scan_data[i-1][0], scan_data[i][0], scan_data[i-1][1], scan_data[i][1], angle);
	        temp_range.push_back(range);
	    }
	    else
	    {
	        temp_range.push_back(std::numeric_limits<float>::infinity());
	    }
	}
	integrated_msg_->ranges = temp_range;
        laser_scan_pub_->publish(*integrated_msg_);

        
    }

    float GET_R(float x, float y){
        return sqrt(x*x + y*y);
    }
    float GET_THETA(float x, float y){
        return atan2(y, x);
    }
    float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle){
        
        return (magnitude_1 + (current_angle - angle_1) * ((magnitude_2 - magnitude_1)/(angle_2 - angle_1)));
    }
    void geometry_quat_to_rpy(double* roll, double* pitch, double* yaw, geometry_msgs::msg::Quaternion geometry_quat){
        tf2::Quaternion quat;
        tf2::convert(geometry_quat, quat);
        tf2::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);  //rpy are Pass by Reference
    }
    void initialize_params(){
        
        this->declare_parameter<std::string>("integratedTopic");
        this->declare_parameter<std::string>("integratedFrameId");

        this->declare_parameter<std::string>("scanTopic1");
        this->declare_parameter<float>("laser1XOff");
        this->declare_parameter<float>("laser1YOff");
        this->declare_parameter<float>("laser1Alpha");
        this->declare_parameter<bool>("show1");

        this->declare_parameter<std::string>("scanTopic2");
        this->declare_parameter<float>("laser2XOff");
        this->declare_parameter<float>("laser2YOff");
        this->declare_parameter<float>("laser2Alpha");
        this->declare_parameter<bool>("show2");

        this->declare_parameter<float>("robotFrontEnd");
        this->declare_parameter<float>("robotRearEnd");
        this->declare_parameter<float>("robotRightEnd");
        this->declare_parameter<float>("robotLeftEnd");
    }
    void refresh_params(){
        this->get_parameter_or<std::string>("integratedTopic", integratedTopic_, "integrated_scan");
        this->get_parameter_or<std::string>("integratedFrameId", integratedFrameId_, "laser");
        this->get_parameter_or<std::string>("scanTopic1",topic1_ ,"lidar_front_right/scan");
        this->get_parameter_or<float>("laser1XOff",laser1XOff_, 0.0);
        this->get_parameter_or<float>("laser1YOff",laser1YOff_, 0.0);
        this->get_parameter_or<float>("laser1Alpha",laser1Alpha_, 0.0);
        this->get_parameter_or<bool>("show1",show1_, true);
        this->get_parameter_or<std::string>("scanTopic2",topic2_, "lidar_rear_left/scan");
        this->get_parameter_or<float>("laser2XOff",laser2XOff_, 0.0);
        this->get_parameter_or<float>("laser2YOff",laser2YOff_, 0.0);
        this->get_parameter_or<float>("laser2Alpha",laser2Alpha_, 0.0);
        this->get_parameter_or<bool>("show2",show2_, false);

        this->get_parameter_or<float>("robotFrontEnd", robotFrontEnd_, 0.0);
        this->get_parameter_or<float>("robotRearEnd", robotRearEnd_, 0.0);
        this->get_parameter_or<float>("robotRightEnd", robotRightEnd_, 0.0);
        this->get_parameter_or<float>("robotLeftEnd", robotLeftEnd_, 0.0);      
    }
    
    std::string topic1_, topic2_, integratedTopic_, integratedFrameId_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1Alpha_;

    float laser2XOff_, laser2YOff_, laser2Alpha_;

    float robotFrontEnd_, robotRearEnd_, robotRightEnd_, robotLeftEnd_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;
    geometry_msgs::msg::TransformStamped trans1_;
    geometry_msgs::msg::TransformStamped trans2_;
    
    double tolerance_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scanMerger>());
    rclcpp::shutdown();
    return 0;
}

