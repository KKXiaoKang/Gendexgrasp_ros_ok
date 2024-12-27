#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection2DArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class ObjectDetectionNode
{
public:
    ObjectDetectionNode()
    {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 订阅目标位置输出
        target_pose_sub_ = nh.subscribe("/target_pose_output", 10, &ObjectDetectionNode::targetPoseCallback, this);

        // 发布物体检测结果
        yolo_segment_result_pub_ = nh.advertise<vision_msgs::Detection2DArray>("/object_yolo_segment_result", 10);
        yolo_tf2_torso_result_pub_ = nh.advertise<vision_msgs::Detection2DArray>("/object_yolo_tf2_torso_result", 10);

        // 初始化TF监听器
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(); 
    }

    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 从消息中获取目标位姿（camera坐标系下的pose）
        geometry_msgs::PoseStamped target_pose = *msg;

        // 这里假设检测的物体是简单的 2D 检测，使用相机坐标系的结果作为检测结果
        vision_msgs::Detection2DArray segment_result;
        segment_result.header = target_pose.header; // 设置时间戳和frame_id
        segment_result.detections.push_back(createDetection(target_pose));

        // 发布相机坐标系下的检测结果
        yolo_segment_result_pub_.publish(segment_result);

        // 广播相机坐标系下的TF转换
        broadcastTransform(target_pose, "camera_color_optical_frame", "camera_object_0");

        // 获取与torso坐标系的转换
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("torso", "camera_color_optical_frame", ros::Time(0));

            // 转换坐标系
            geometry_msgs::PoseStamped target_pose_torso;
            tf2::doTransform(target_pose, target_pose_torso, transform);

            // 创建目标物体检测结果（在 torso 坐标系下）
            vision_msgs::Detection2DArray tf2_torso_result;
            tf2_torso_result.header = target_pose_torso.header;
            tf2_torso_result.detections.push_back(createDetection(target_pose_torso));

            // 发布 torso 坐标系下的检测结果
            yolo_tf2_torso_result_pub_.publish(tf2_torso_result);

            // 广播 torso 坐标系下的TF转换
            broadcastTransform(target_pose_torso, "torso", "torso_object_0");
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF Error: %s", ex.what());
        }
    }

    vision_msgs::Detection2D createDetection(const geometry_msgs::PoseStamped &pose)
    {
        // detection返回值
        vision_msgs::Detection2D detection;
        detection.header = pose.header;

        // hypothesis组合
        vision_msgs::ObjectHypothesisWithPose hypothesis;
        hypothesis.id = 0; 
        hypothesis.score = 1.0;
        hypothesis.pose.pose.position = pose.pose.position;
        hypothesis.pose.pose.position.x -= 0.05; //物体往右偏移5cm
        hypothesis.pose.pose.position.y -= 0.05; //物体往右偏移5cm
        hypothesis.pose.pose.orientation = pose.pose.orientation;
        
        // hypothesis.pose.pose.orientation.x = 0.0;
        // hypothesis.pose.pose.orientation.y = 0.0;
        // hypothesis.pose.pose.orientation.z = 0.0;
        // hypothesis.pose.pose.orientation.w = 1.0;

        detection.results.push_back(hypothesis);

        // 填充像素坐标系信息
        detection.bbox.center.x = 0.1;
        detection.bbox.center.y = 0.1;
        detection.bbox.size_x = 0.1;  // 假设物体的尺寸
        detection.bbox.size_y = 0.1;  // 假设物体的尺寸
        return detection;
    }


    void broadcastTransform(const geometry_msgs::PoseStamped &pose, const std::string &target_frame, const std::string &child_frame)
    {
        // 创建Transform消息
        geometry_msgs::TransformStamped transform_stamped;

        // 设置消息头
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = target_frame;
        transform_stamped.child_frame_id = child_frame;  // 目标物体的frame_id

        // 填充位置信息
        transform_stamped.transform.translation.x = pose.pose.position.x;
        transform_stamped.transform.translation.y = pose.pose.position.y;
        transform_stamped.transform.translation.z = pose.pose.position.z;

        // 填充方向信息
        transform_stamped.transform.rotation = pose.pose.orientation;

        // 广播变换
        tf_broadcaster_->sendTransform(transform_stamped);
    }

private:
    ros::Subscriber target_pose_sub_;
    ros::Publisher yolo_segment_result_pub_;
    ros::Publisher yolo_tf2_torso_result_pub_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detection_node");
    ObjectDetectionNode node;
    ros::spin();
    return 0;
}
