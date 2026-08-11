#include <chrono>  
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;


class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        /*TODO TASK - MILESTONE # 4.1
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */
        // Declare parameters
        this->declare_parameter<double>("following_distance", .5);
        this->declare_parameter<int64_t>("wall_side", -1);

        this->declare_parameter<double>("buffer_zone", 1.0);
        this->declare_parameter<double>("forward_velocity", 0.2);

        this->declare_parameter<double>("angle_control_gain_1", 2.0);
        this->declare_parameter<double>("angle_control_gain_2", 2.0);

        // Get parameter values
        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);

        // Print parameter values
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %ld", wall_side_);

        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);

        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);

        
        /* TODO TASK - MILESTONE #4.3
        Initialise dynamic parameter handler by the rclcpp node method "add_on_set_parameters_callback"
        */
       //  Initalise the dynamic parameter handler
       dyn_params_handler_ = this->add_on_set_parameters_callback(
           std::bind(
           &WallFollower::dynamicParametersCallback,
           this, std::placeholders::_1));

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
             "/cmd_vel",
             rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    /* TODO TASK - MILESTONE #4.2
        define dynamic parameter call back handle.
    */

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    double distance_control_gain_;
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    /*TODO TASKS
        MILESTONE # 6.1. Process the received scan_msg to get the location of the closest object in robot's environment. 
        NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance 
        measurements of these four pillars by ignoring any measurement less than 0.2 meter. 
    */


    auto range_min = scan_msg->range_min;
    auto range_max = scan_msg->range_max;

    auto angle_min = scan_msg->angle_min;
    auto angle_max = scan_msg->angle_max;
    auto angle_increment = scan_msg->angle_increment;

    auto ranges = scan_msg->ranges;
    auto intensities = scan_msg->intensities;

    int measurements_count = ranges.size();

    float min_range_measurement = std::numeric_limits<float>::infinity();
    int min_range_measurement_index = -1;

    for (int i = 0; i < measurements_count; i++) {
      if (ranges[i] > range_min && ranges[i] < range_max) {
        if (ranges[i] < min_range_measurement && ranges[i] > 0.2) {
          min_range_measurement = ranges[i];
          min_range_measurement_index = i;
        }
      }
    }
    // RCLCPP_INFO(this->get_logger(), "min range: %.2f", min_range_measurement);
    

    /*
        MILESTONE # 6.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have         
        to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to  the robot's coordinate.

    */
    
    if (min_range_measurement_index == -1) {
        RCLCPP_INFO(this->get_logger(), "No valid measurements found.");
        return;
    } else if (min_range_measurement > 12) {
        RCLCPP_INFO(this->get_logger(), "No objects found within range.");
        return;
    }

    float min_range_measurement_angle = angle_min + min_range_measurement_index * angle_increment;
    // RCLCPP_INFO(this->get_logger(), "Closest object at range: %.2f m, angle: %.2f rad", min_range_measurement, min_range_measurement_angle);
    if (min_range_measurement_angle < angle_min || min_range_measurement_angle > angle_max) {
        RCLCPP_WARN(this->get_logger(), "Calculated angle is outside of the allowed sensor range.");
    }

    // Orient to robot frame
    min_range_measurement_angle -= PI/2;
    
    /*
        MILESTONE # 6.3. Write a Wall Follow Reactive Control that takes the bearing and range information of the closest object in the environment 
        as the input and publish a message on topic /cmd_vel to control the motion of the robot. 
            3.1 If the robot is far away from the wall, it should move towards its nearest wall at a constant speed until the robot 
            arrives at a distance of desired value + buffer zone, with respect to its closest wall. 
            3.2 Next, the robot enter the wall follow mode with the control lawy in in Algorithm 1
            3.3 The robot should deal with corner cases by only using reactive control with properly tuned control gains. 
    */

    geometry_msgs::msg::Twist cmd_vel;


    double wall_angle;
    if (wall_side_ == 1) {
        wall_angle = min_range_measurement_angle - PI / 2;  
    }  else {
        wall_angle = min_range_measurement_angle + PI / 2;  

    }
    // double wall_angle = min_range_measurement_angle;
    double wall_distance_error = min_range_measurement - following_distance_;
    // double wall_distance_error = 0;
    
    
    // Handle wrapping
    while (wall_angle < -PI) {
        wall_angle += 2 * PI;
    }
    
    while (wall_angle > PI) {
        wall_angle -= 2 * PI;
    }
    RCLCPP_INFO(this->get_logger(), "Wall Distance Error: %.2f m, Wall Angle: %.2f rad", wall_distance_error, wall_angle);
    
    if (wall_side_ == 1) {
        if (abs(wall_angle) > PI / 10) {
            cmd_vel.angular.z = angle_control_gain_1_ * wall_angle + angle_control_gain_2_ * wall_distance_error * sin(wall_angle) / wall_angle;
        } else {
            cmd_vel.angular.z = angle_control_gain_1_ * wall_angle + angle_control_gain_2_ * wall_distance_error;
        }
    } else {
        if (abs(wall_angle) > PI / 10) {
            cmd_vel.angular.z = angle_control_gain_1_ * wall_angle - angle_control_gain_2_ * wall_distance_error * sin(wall_angle) / wall_angle;
        } else {
            cmd_vel.angular.z = angle_control_gain_1_ * wall_angle - angle_control_gain_2_ * wall_distance_error;
        }
    }

    cmd_vel.linear.x = forward_velocity_;
    if (cmd_vel.linear.x < 0.0) {
        cmd_vel.linear.x = 0.0;
        RCLCPP_INFO(this->get_logger(), "Attempted to move backwards, setting linear velocity to 0.");

    }

    cmd_vel_publisher_->publish(cmd_vel);
   
}


rcl_interfaces::msg::SetParametersResult 
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    /*TODO TASK - MILESTONE #5.1 
      Check whether update of a parameter in the node is requested, if yes and save the updated
      parameter value.
    */
    for (auto parameter : parameters) {
        const auto & param_type = parameter.get_type();
        const auto & param_name = parameter.get_name();
        if (param_type == ParameterType::PARAMETER_DOUBLE) {

            if (param_name == "following_distance") {
                following_distance_ = parameter.as_double();
                if(following_distance_ < 0.2)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set following_distance to to close to the wall ( < 0.2m),"
                    " this isn't allowed, so the distance will be set to be 0.2.");
                    following_distance_ = 0.2;
                }
            }

            if (param_name == "buffer_zone") {
                buffer_zone_ = parameter.as_double();
                if(buffer_zone_ < 0.2)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set buffer_zone to to close to the wall ( < 0.2m),"
                    " this isn't allowed, so the distance will be set to be 0.2.");
                    buffer_zone_ = 0.2;
                }
            }


            if (param_name == "forward_velocity") {
                forward_velocity_ = parameter.as_double();
                if(forward_velocity_ < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set forward_velocity to negative"
                    " this isn't allowed, so the velocity will be set to be 0.");
                    forward_velocity_ = 0;
                }
            }


            if (param_name == "angle_control_gain_1") {
                angle_control_gain_1_ = parameter.as_double();
                if(angle_control_gain_1_ < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_1 to negative"
                    " this isn't allowed, so the gain will be set to be 0.");
                    angle_control_gain_1_ = 0;
                }
            }



            if (param_name == "angle_control_gain_2") {
                angle_control_gain_2_ = parameter.as_double();
                if(angle_control_gain_2_ < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_2 to negative"
                    " this isn't allowed, so the gain will be set to be 0.");
                    angle_control_gain_2_ = 0;
                }
            }


        }

        if (param_type == ParameterType::PARAMETER_INTEGER) {
            if (param_name == "wall_side") {
                wall_side_ = parameter.as_int();
                if(wall_side_ == 1 || wall_side_ == -1)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set wall_side to neither -1, or 1"
                    " this isn't allowed, so the side will be set to be 1.");
                    wall_side_ = 1;
                }
            }

        }


    }
  result.successful = true;
  return result;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}