#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include <string>

class manager {

  public:

    manager(ros::NodeHandle& In_nh);
    ~manager(void);
    void run (void);
  

  private:

    void init (void);
    void step( void );
    void connect_to_neighbours(void);
    void handle_neighbour_1_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos);
    void handle_neighbour_2_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos);
    void handle_sim_enable (const std_msgs::Int32::ConstPtr& new_sim_enable);
    void handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position);
    void wait (void);

    ros::NodeHandle nh;
    ros::Subscriber neighbour_1_sub;
    ros::Subscriber neighbour_2_sub;
    ros::Subscriber position_sub;
    ros::Subscriber stop_sim_sub;
    ros::Publisher status_pub;
    
    std_msgs::Int32 stop_count_msg;
    std::string neighbour_1;
    std::string neighbour_2;

    float x_neighbour_1,x_neighbour_2, x;
    int RUN_ENABLE;

    ros::Time LastTimestamp;
    double dt;




};