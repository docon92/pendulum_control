#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"

class controller {

  public:

    controller(  ros::NodeHandle& In_nh,float rate);
    ~controller(void);
    void run (void);
  

  private:

    void step( void );
    float saturate_output (float cmd);
    void handle_run_enable (const std_msgs::Int32::ConstPtr& new_flag);
    void handle_pendulum_stop (const std_msgs::Int32::ConstPtr& new_flag);
    void handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position);
    void handle_velocity (const geometry_msgs::Pose2D::ConstPtr& new_velocity);
    float sign_of_num(float num);

    ros::NodeHandle nh;
    ros::Subscriber position_sub;
    ros::Subscriber run_enable_sub;
    ros::Subscriber pendulum_stop_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher force_output_pub;
    ros::Rate LoopRate;
    
    std_msgs::Float64 Force_msg;


    float k1, k2, k3, k4, k5;
    float x1, x2, x3, x4, x3i, x3d, x3_0;
    float Fd, F, max_F;
    bool CONTROLLER_INIT;
    int RUN_ENABLE, PENDULUM_STOP;

    ros::Time LastTimestamp;
    double dt;




};