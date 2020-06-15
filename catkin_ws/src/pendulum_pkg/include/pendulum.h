#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include <time.h>

class pendulum {

  public:

    pendulum(ros::NodeHandle& In_nh, float rate);
    ~pendulum(void);
    void run (void);
  
  private:

    void init (void);
    void step( void );
    void calculate_disturbance (void);
    void handle_run_enable (const std_msgs::Int32::ConstPtr& new_flag);
    void handle_pendulum_stop (const std_msgs::Int32::ConstPtr& new_flag);
    void handle_force_input (const std_msgs::Float64::ConstPtr& new_force_input);
    void broadcast_state( void );
    
    ros::NodeHandle nh;
    ros::Subscriber run_enable_sub;
    ros::Subscriber pendulum_stop_sub;
    ros::Subscriber force_input_sub;
    ros::Publisher position_pub;
    ros::Publisher velocity_pub;
    ros::Rate LoopRate;
    
    geometry_msgs::Pose2D velocity_msg;
    geometry_msgs::Pose2D position_msg;

    std::string neighbour_1;
    std::string neighbour_2;

    float x1, x2,delta_x1, delta_x2, x3, x4,delta_x3, delta_x4;
    float l, m, M;
    float u, F, disturbance, max_disturbance;
    float c1,c2,c3,c4;
    float sim_rate;
    float sim_rate_default;
    int RUN_ENABLE, PENDULUM_STOP;
    int rand_seed;

    ros::Time LastTimestamp;
    double dt;

};