// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include<map_creator/hdf5_dataset.h>
#include <boost/format.hpp>

//struct stat st;

typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;

//static const std::string service_setting = "ALL"; // 定义要生成工作空间的类型
static const std::string service_setting = "SPECIFIED"; 

/**
 * @brief isFloat 函数用于判断一个字符串是否能转换为浮点数
 * @param s 字符串
 * @return 能否转换为浮点数
*/
bool isFloat(std::string s)
{
  std::istringstream iss(s);
  float dummy;
  iss >> std::noskipws >> dummy;
  return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace"); // 初始化ROS节点，节点名称为"workspace"
  ros::NodeHandle n;  // 创建一个节点句柄，用于与ROS通信
  ros::Time startit = ros::Time::now();   // 记录程序开始运行的时间
  float resolution = 0.08;   // 默认设置分辨率为0.08米 - 用于初始化体素盒子建立姿态合成数据
  kinematics::Kinematics k;  // 创建一个Kinematics对象，用于后续的运动学计算
  std::string file = str(boost::format("%s_r%d_reachability.h5") % k.getRobotName() % resolution); // 使用Boost的格式化功能生成文件名，其中包含了机器人的名字和分辨率
  std::string path(ros::package::getPath("map_creator") + "/maps/");  // 获取ROS包的路径，并构造存储地图文件的路径
  std::string filename;   // 定义保存文件的名称

  // 节点参数传入check检查
  if (argc == 2)
  {
    if (!isFloat(argv[1]))
    {
      ROS_ERROR_STREAM("Probably you have just provided only the map filename. Hey!! The first argument is the "
                       "resolution.");
      return 0;
    }
    resolution = atof(argv[1]);
    file = str(boost::format("%s_r%d_reachability.h5") % k.getRobotName() % resolution);
    filename = path + file;
  }

  else if (argc == 3)
  {
    std::string name;
    name = argv[2];
    if (!isFloat(argv[1]) && isFloat(argv[2]))
    {
      ROS_ERROR_STREAM("Hey!! The first argument is the resolution and the second argument is the map filename. You "
                       "messed up.");
      return 0;
    }

    else
    {
      resolution = atof(argv[1]);
      std::string str(argv[2]);
      if(std::strchr(str.c_str(), '/'))
      {
        filename = argv[2];
      }
      else
        filename = path + str;
    }
  }
  else if (argc < 2)
  {
    ROS_INFO("You have not provided any argument. So taking default values.");
    filename = path + file;
  }
  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("workspace", 10);
  ros::Rate loop_rate(10); // 设置循环频率为10Hz
 
  int count = 0; // 定义一个计数器

  while (ros::ok())
  {
    /**
     * 确认octomap八叉树的体素（Voxel）的分辨率层级
     */ 
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    /**
     * 创建一个半径为1的盒子。这个盒子的大小将是机器人大小加上1.5。然后，这个盒子被指定分辨率的体素离散化。 
    */
    sphere_discretization::SphereDiscretization sd; // 实例化一个 SphereDiscretization 对象 sd，用于将空间进行离散化
    float r = 1; // 设置一个球的半径 r=1
    octomap::point3d origin = octomap::point3d(0, 0, 0);  // 机器人的基座设为原点 (0, 0, 0)
    // 使用 generateBoxTree 方法，生成一个以机器人基座为中心、半径为 r 的体素盒子，并根据指定的分辨率对其进行离散化
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< octomap::point3d > new_data; // 创建一个 new_data 容器用于存储生成的体素的中心点。
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution); // 打印出信息，表明正在使用指定的分辨率创建盒子
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      sphere_count++; // 计算体素盒子中所有叶节点的数量，并保存到 sphere_count 中
    }
    new_data.reserve(sphere_count); // 为 new_data 预留足够的空间
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      new_data.push_back(it.getCoordinate()); // 并将每个体素叶节点的坐标存储到 new_data 中
    }

    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    /**
     * 在每个体素中创建一个球体。球体可能是通过默认方式或其他技术创建的。 
     * TODO 需要修改其他技术。用户可以指定他们想要使用的创建技术。 
     * TODO 球体的离散化参数和每个姿势的旋转将被作为参数传入。如果最终的关节可以在(0, 2π)范围内旋转，我们就不需要对姿势进行旋转。 
     * 球体上每个离散化点都被转换为姿势，并且所有姿势都保存在一个多映射中，与它们对应的球心相关联。 
     * 如果分辨率是0.01，程序将不会响应。
    */
    float radius = resolution; // 设置球的半径为分辨率大小

    VectorOfVectors sphere_coord; // 存储每个球的坐标
    sphere_coord.resize( new_data.size() ); // sphere_coord的大小和 new_data 相同，用于存储每个球的坐标

    MultiVector pose_col; // 用于存储姿态与球心的配对信息
    pose_col.reserve( new_data.size() * 50); // ?

    for (int i = 0; i < new_data.size(); i++)
    {
      static std::vector< geometry_msgs::Pose > pose;        // 创建一个ros通用的geometry_msgs的Pose向量
      sd.convertPointToVector(new_data[i], sphere_coord[i]); // 通过 convertPointToVector 方法，将球的中心转换成坐标存储在 sphere_coord

      // 检查服务设置
      if (service_setting == "ALL") {
          // 生成所有可能的姿态
          sd.make_sphere_poses(new_data[i], radius, pose);
          // 产生全部姿态
          ROS_INFO("Generating all possible poses for sphere %d", i);
      }else if (service_setting == "SPECIFIED") {
          // 生成特定姿态（这里需要定义一个特定的姿态向量或四元数）
          std::vector<tf2::Quaternion> specific_orientations;  // 存储所有特定姿态的四元数
          
          // 从提供的数据中创建四元数
          // contactdb+water_bottle_170_best_q [ 0.63951406 -0.32413632  0.60869027  0.33978459]
          // contactdb+water_bottle_126_best_q [ 0.72399718  0.19540314  0.64770873 -0.13460719]
          tf2::Quaternion specific_orientation;
          // water_bottle_126_best_q
          specific_orientation.setX(0.7201371761882054);
          specific_orientation.setY(0.19436134185739312);
          specific_orientation.setZ(0.6442554546357139);
          specific_orientation.setW(-0.13388952575544086);     

          // // // water_bottle_170_best_q
          // specific_orientation.setX(0.63951406);
          // specific_orientation.setY(-0.32413632);
          // specific_orientation.setZ(0.60869027);
          // specific_orientation.setW(0.33978459); 

          // 将特定的姿态添加到向量中
          specific_orientations.push_back(specific_orientation);

          // 调用函数生成特定姿态的集合     
          sd.make_sphere_specific_poses(new_data[i], radius, specific_orientations, pose);
          
          // 产生全部姿态
          ROS_INFO("Generating SPECIFIED possible poses for sphere %d", i);
      }

      for (int j = 0; j < pose.size(); j++)
      {
        static std::vector< double > point_on_sphere;
        sd.convertPoseToVector(pose[j], point_on_sphere); // 将姿势转换为向量，存储在point_on_sphere当中
        pose_col.push_back( std::make_pair(point_on_sphere, &sphere_coord[i])); // 并与相应的球体配对存储在 pose_col（里面代表了体素盒子内的所有点+姿态的集合）
      }
    }
    
    /**
     * 检查每个姿势的反向运动学（IK）解。可到达的姿势及其对应的关节解被存储起来。 
     * 只存储第一个关节解。我们可能在将来需要这些解。否则，我们可以在一个并行线程中用这些关节解展示机器人跳舞。 
     * TODO 需要实现支持超过6自由度（6DOF）机器人的功能
     * Kinematics k;
    */
    MultiMapPtr pose_col_filter;  // 创建一个过滤后的多映射 pose_col_filter 用于存储可到达的姿势
    VectorOfVectors ik_solutions; // ik_solutions存储逆向运动学解
    ik_solutions.reserve( pose_col.size() ); // ik_solutions的大小和 pose_col 相同，用于存储逆向运动学解

    for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {
      static std::vector< double > joints(6); // 初始化结果为6自由度的解
      int solns;
      if (k.isIKSuccess(it->first, joints, solns)) // ik结果向量joints | 可行解的数量 solns
      {
        pose_col_filter.insert( std::make_pair( it->second, &(it->first))); // it->second 是指向球体中心的坐标数据 | &(it->first)存储的是该姿势的地址
        ik_solutions.push_back(joints); // 存储关节解的向量集合
        // cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<" "<<it->first[5]<<" "<<it->first[6]<<endl;
      }
    }
    

    ROS_INFO("Total number of poses: %lu", pose_col.size()); // 打印出总姿势数量
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size()); // 打印可到达的姿势数量。

    /**
     * 根据可到达的姿态数量 | 计算每个体素球的灵巧度
     * 可达球体的中心被存储在一个映射（map）中。这个数据将被用于在可视化器中展示球体。 // TODO 有几个映射已经被实现。我们可以去掉一些映射并运行较大的循环。访问映射的复杂度是O(log(n))
    */
    MapVecDoublePtr sphere_color;

    for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      const std::vector<double>* sphere_coord    = it->first;
      //const std::vector<double>* point_on_sphere = it->second;

      // 计算每个球体的可达性指数 D=R/N*100，并将其与球体坐标配对存储在 sphere_color 中
      // Reachability Index D=R/N*100;
      float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / new_data.size()) * 100;
      sphere_color.insert( std::make_pair(it->first, double(d)));
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size()); // 打印出可到达的球体数量

    // Creating maps now
//Saving map to dataset
    hdf5_dataset::Hdf5Dataset h5(filename);
    h5.saveReachMapsToDataset(pose_col_filter, sphere_color, resolution); // 将可达的姿势和球体颜色数据保存到 HDF5 数据集中

    double dif = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed"); // 计算并打印整个过程耗时，表示完成
    ros::spinOnce();
    // sleep(10000);
    return 1;
    loop_rate.sleep();
    count; // 执行一次 ROS 回调，并按照设定的频率让程序暂停一段时间
  }
  return 0;
}
