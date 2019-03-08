#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf::TransformBroadcaster br;
    // 定义两个坐标系之间的变换关系
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);

    // 广播坐标变换: 这个广播如果停止了，listener那边就会找不到这两个坐标系的名字
    // 一开始把这里的turtle_name写成了“turtle_name”，结果rviz里面就有一个叫做turtle_name
    // 的坐标系，listener里面lookupTransform需要查询的是名为turtle1的坐标系，这边写错了就查不到了
    // 这边只有把这个坐标系的状态广播出去，别的地方才能监听到这个坐标系的位姿
    br.sendTransform(
    tf::StampedTransform(
        transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    if(argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1]; // argv[1]能够提取小海龟的名字

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}