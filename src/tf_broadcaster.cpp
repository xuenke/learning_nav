#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  // TransformBroadcaster 用于publish transforms
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

      // 定义两个坐标系之间的变换关系
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(3.0, 0.0, 0.0));
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);

      // 广播坐标变换: 这个广播如果停止了，listener那边就会找不到这两个坐标系的名字
      broadcaster.sendTransform(
        tf::StampedTransform(
          transform, ros::Time::now(), "base_link", "base_laser"));

    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
    //     ros::Time::now(),"base_link", "base_laser"));

    r.sleep();
  }
}
