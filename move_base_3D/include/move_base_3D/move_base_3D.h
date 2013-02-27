#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <sbpl_3d_planner/sbpl_3d_planner.h>

class MoveBase3D
{

  public:
    MoveBase3D();
    void getCollisionMap(arm_navigation_msgs::CollisionMap& collision_map);
    void getCurrentPosition(geometry_msgs::PoseStamped& current_pos);
    virtual ~MoveBase3D();

  private:
    ros::NodeHandle nh;
    ros::Subscriber SLAM_sub; /** Subscriber for updating current position from SLAM */
    ros::Subscriber goal_sub; /** Subscriber for the goal topic */
    ros::Subscriber collision_map_sub; /** Subscriber for the Octomap collision map */

    /* TODO: Make this a parameter. Like an action lib plugin.*/
    SBPL3DPlanner* global_planner;  /** Instance of the global planner class*/


    void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal); /** Callback for the goal topic*/
    void SLAMCallback(const geometry_msgs::PoseStampedConstPtr& msg); /** Callback to update current position from SLAM */
    void collisionMapCallback(const arm_navigation_msgs::CollisionMap& collision_map_in); /** Callback for the collision map topic*/
    
    geometry_msgs::PoseStamped currentPos_;

    arm_navigation_msgs::CollisionMap collision_map;


    /* ROS Params */
    std::string static_collision_map_;/** Set to non-empty string if loading a pre-built collision map (.bt file)*/
    double local_pub_rate_;   /** Local window collision map publisher rate*/
    double window_side_;  /** Side length of the local collision map cube */
};
