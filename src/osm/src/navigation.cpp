#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

rclcpp::Node::SharedPtr nh; 
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_pub;
bool flag_next = false;   
bool flag_new = false;
float goal_x = 0.0 ,goal_y = 0.0;
int num = 0, i = 0;

void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) 
{
    
    RCLCPP_INFO(nh->get_logger(), "%d",msg->poses.size());
    num = msg->poses.size();
    
    goal_x = msg->poses[i].pose.position.x;
    goal_y = msg->poses[i].pose.position.y;
    
    flag_new = true;
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    if(!flag_new)
    {
    	if(abs(goal_x - x) < 1.0 && abs(goal_y - y) < 1.0)
    	    flag_next = true;
    }

}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("navigation") ;
    rclcpp::Clock clock;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
     
    path_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("goal",100);
    
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>("odom",1,odomCallback);
    path_sub = nh->create_subscription<nav_msgs::msg::Path>("shortest_path",1,pathCallback);
    
    while(rclcpp::ok())
    {
    
    	if(flag_next | flag_new)
    	{
    	    flag_new = false;
    	    i = i+1;
    	    flag_next = false;
    	
	    geometry_msgs::msg::PoseStamped goal;
	    goal.pose.position.x = goal_x;
	    goal.pose.position.y = goal_y;
	    goal.header.frame_id = "map";
	    path_pub->publish(goal);
	}    
    
    	if(i == num-1)
    	{
    	    i = 0;
    	    flag_new = false;
    	    flag_next = false;
    	    
    	}
	rclcpp::spin_some(nh);
    }

    return 0;
}
