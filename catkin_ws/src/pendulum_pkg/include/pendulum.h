#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"

class pendulum {

  public:

    pendulum(  ros::NodeHandle& In_nh,
             float theta_0,
             float x_0,
             float length,
             float m_pendulum,
             float M_cart,
             float max_dist);
    ~pendulum(void);
    void run (void);
  

  private:

    void step( void );
    void calculate_disturbance (void);
    void handle_force_input (const std_msgs::Float64::ConstPtr& new_force_input);


    ros::NodeHandle nh;
    ros::Subscriber force_input_sub;
    /* Publish the NED position */
    ros::Publisher position_pub;
    ros::Publisher velocity_pub;
    
    geometry_msgs::Pose2D velocity_msg;
    geometry_msgs::Pose2D position_msg;

    float x1, x2,delta_x1, delta_x2, x3, x4,delta_x3, delta_x4;
    float l;
    float m;
    float M;
    float u, F, disturbance, max_disturbance;
    float c1,c2,c3,c4;

    ros::Time LastTimestamp;
    double dt;




};