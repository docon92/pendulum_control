#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"


class pendulum {

  public:

    pendulum(ros::NodeHandle& In_nh);
    ~pendulum(void);
    void run (void);
  

  private:

    void init (ros::NodeHandle& In_nh);
    void step( void );
    void calculate_disturbance (void);
    void handle_force_input (const std_msgs::Float64::ConstPtr& new_force_input);
    void wait (void);

    ros::NodeHandle nh;
    ros::Subscriber force_input_sub;
    ros::Subscriber neighbour_1_sub;
    ros::Subscriber neighbour_2_sub;
    ros::Publisher position_pub;
    ros::Publisher velocity_pub;
    
    geometry_msgs::Pose2D velocity_msg;
    geometry_msgs::Pose2D position_msg;

    std::string neighbour_1;
    std::string neighbour_2;

    float x1, x2,delta_x1, delta_x2, x3, x4,delta_x3, delta_x4;
    float l;
    float m;
    float M;
    float u, F, disturbance, max_disturbance;
    float c1,c2,c3,c4;

    ros::Time LastTimestamp;
    double dt;




};