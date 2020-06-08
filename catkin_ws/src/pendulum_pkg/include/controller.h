#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"

class controller {

  public:

    controller(  ros::NodeHandle& In_nh,
                 float in_k1,
                 float in_k2,
                 float in_k3,
                 float in_k4,
                 float in_k5,
                 float in_max_output);
    ~controller(void);
    void run (void);
  

  private:

    void step( void );
    float saturate_output (float commanded_force);
    void handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position);
    void handle_velocity (const geometry_msgs::Pose2D::ConstPtr& new_velocity);


    ros::NodeHandle nh;
    ros::Subscriber position_sub;
    ros::Subscriber velocity_sub;
    
    /* Publish the NED position */
    ros::Publisher force_output_pub;
    
    std_msgs::Float64 Force_msg;


    float k1, k2, k3, k4, k5;
    float x1, x2, x3, x4, x3i;
    float F, max_F;

    ros::Time LastTimestamp;
    double dt;




};