#include <iostream>
#include <iostream>
 
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/types_slam3d.h"
//#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/stuff/command_args.h"
 
#include "Eigen/Core"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <chrono>
 // for ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// path
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
 
using namespace std;
using namespace g2o;
 
//先定于曲线模型的顶点模板
//tarmy function
/*
void displayMkakers(visualization_msgs::Marker &makers)
{
makers.pose.position.x = 0;
 makers.pose.position.y = 0;
}
*/
//<3表示一个顶点中待优化的参数个数，Eigen::Vector3d表示这些待估计变量放在一个顶点中的存储类型>
class CurveFittingVertex:public g2o::BaseVertex<3,Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() //纯虚函数重载
    {
        _estimate<<0,0,0;
    }
 
    virtual void oplusImpl(const double* update) //更新
    {
        _estimate+=Eigen::Vector3d(update);
    }
    // 存和读 纯虚函数
    virtual bool read(istream &in){}
    virtual bool write(ostream &out) const {}
};
 
//假设曲线模型为 y=ax^2+bx+c
//定义边的模板 :观测值维度，类型，连接顶点类型
class CurveFittingEdge:public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x):BaseUnaryEdge(),_x(x){} //对父类BaseUnaryEdge初始化，同时初始化成员变量_x
    //误差函数模型 重载
    void computeError(){
        //_vertices是VertexContainer顶点容器类型
        const CurveFittingVertex* v=static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc=v->estimate();
        //abc(0,0)表示[v1,v2,v2]^T中的v1
        //abc(1,0)表示[v1,v2,v2]^T中的v2
        //abc(2,0)表示[v1,v2,v2]^T中的v3
        //误差则为测量值y_hat-y 这里为什么有个exp?
        _error(0,0)=_measurement-std::exp(abc(0,0)*_x*_x+abc(1,0)*_x+abc(2,0));
    }
    virtual bool read(istream &in){}
    virtual bool write(ostream &out) const{}
public:
    double _x; //x值
};
int main(int argc, char **argv)
{
   ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher person_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    nav_msgs::Path gui_path;
    //nav_msgs::Path path;
    gui_path.header.stamp=ros::Time::now();
    gui_path.header.frame_id="gui_path";
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
   
 cout << "Hello G2O" << endl;
 
    //定义曲线真实参数
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点个数
    double w_sigma=0.1;                 //噪声
    cv::RNG rng;                        //opencv随机数
    //double abc[3]={0,0,0};              //abc参数（待优化值）
 
    vector<double> x_data,y_data;
 
    cout<<"生成数据:"<<endl;
    for(int i=0;i<N;i++){
        double x=i/100.0;
        x_data.push_back(x);
        y_data.push_back(
                    exp(a*x*x+b*x+c)+rng.gaussian(w_sigma));
        cout<<x_data[i]<<"  "<<y_data[i]<<endl;
    }
 
    //创建线性求解器
    auto linearSolver=g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();
//tarmy new
 //std::unique_ptr<Block::LinearSolverType> linearSolver ( new LinearSolverCSparse<BlockSolverX::PoseMatrixType>() );
    //创建块求解器
    auto blockSolver=g2o::make_unique<BlockSolverX>(std::move(linearSolver));
//tarmy new
//std::unique_ptr<BlockSolverX> blockSolver (new BlockSolverX( std::move(linearSolver) ));
 
    //选择优化方法
    OptimizationAlgorithmLevenberg * optimizationAlgorithm=
            new OptimizationAlgorithmLevenberg(std::move(blockSolver));
 
    //创建图模型 设置优化方法
    SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);
 
    //加入顶点
    CurveFittingVertex*v =new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0,0,0));
    v->setId(0);
    optimizer.addVertex(v);
 
    //加入边
    for(int i=0;i<N;i++){
        //使用模拟的x值构造边对象
        CurveFittingEdge *edge=new CurveFittingEdge(x_data[i]);
        //边的id
        edge->setId(i);
        //边的链接对象，这里只添加了一个顶点v，因此另一端链接起始顶点0
        edge->setVertex(0,v);
        //设定边的观测值,即赋值_measurement=y_data[i]
        edge->setMeasurement(y_data[i]);
        // 信息矩阵：协方差矩阵之逆
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) );
        //添加边
        optimizer.addEdge(edge);
    }
 
    //开始优化
    cout<<"开始执行优化"<<endl;
    chrono::steady_clock::time_point t1=chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);//迭代,参数为迭代次数
    chrono::steady_clock::time_point t2=chrono::steady_clock::now();
    chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
 
    //输出优化值
    Eigen::Vector3d abc_estimate=v->estimate();
    cout<<"a b c is"<<abc_estimate.transpose()<<endl;
   // return 0;


    ros::Rate loop_rate(50);
    while (ros::ok())
    {

    int xyindex;
        visualization_msgs::Marker marker;
        geometry_msgs::Vector3 xypoint;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_data[xyindex];
    marker.pose.position.y = y_data[xyindex];
// point of path
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = x_data[xyindex];;
        this_pose_stamped.pose.position.y = y_data[xyindex];
        this_pose_stamped.header.stamp=ros::Time::now();
        this_pose_stamped.header.frame_id="pose_stamped";
        gui_path.poses.push_back(this_pose_stamped);


    xyindex++;
    if(xyindex>200)xyindex=0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      //ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    person_pub.publish(gui_path);
     r.sleep();
    }
}
