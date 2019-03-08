#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = 
        node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    ros::Publisher turtle_vel = 
        node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    
    // TransformListener用于接受已经发布的坐标变换：能够缓存10s
    // 啥意思：The TransformListener object should be scoped to persist 
    // otherwise it's cache will be unable to fill and almost every query will fail
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while(node.ok()){
        // 定义一个transform，用来存储监听到的坐标变换
        tf::StampedTransform transform;
        try{
            // 监听两个海龟之间的坐标变换并存入transform中，time(0)表示获得最近的坐标变换
            // 跟随方法1：等待坐标变换，至多等3s。
            ros::Time now = ros::Time::now();
            listener.waitForTransform("/turtle2", "turtle1", now, ros::Duration(3.0));
            listener.lookupTransform("/turtle2", "/turtle1", now, transform);
            // 跟随方法2：我们可以lookuptime(0)，去获得最近的坐标变换，这样做没问题
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
            // 跟随方法3：我们让海龟2跟随海龟1在5s钟之前的位置
            ros::Time past = ros::Time::now() - ros::Duration(5.0);
            listener.waitForTransform("/turtle2", "/turtle1",
                                      past, ros::Duration(1.0));
            listener.lookupTransform("/turtle2", "/turtle1",
                                      past, transform);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);

        rate.sleep();
    }
    return 0;
}