#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/core/Mapper.hpp>

using namespace std::chrono_literals;

timeval fromRosTime(const rclcpp::Time& rt)
{
	timeval t;
	t.tv_sec = rt.seconds();

	rcl_time_point_value_t diff = t.tv_sec * 1000000000;
	rcl_time_point_value_t nsec = rt.nanoseconds() - diff;
	t.tv_usec = nsec / 1000;
	return t;
}

rclcpp::Time fromTimeval(const timeval& tv)
{
	return rclcpp::Time(tv.tv_sec, tv.tv_usec * 1000, RCL_CLOCK_UNINITIALIZED);
}

class RosClock : public slam3d::Clock
{
public:
	RosClock(rclcpp::Node* n) : mNode(n){}

	virtual timeval now()
	{
		return fromRosTime(mNode->now());
	}
private:
	rclcpp::Node* mNode;
};

class TfOdometry : public slam3d::PoseSensor
{
public:
	TfOdometry(slam3d::Graph* g, slam3d::Logger* l, tf2_ros::Buffer* tf)
	: slam3d::PoseSensor("Odometry", g, l), mTfBuffer(tf){}
	
	slam3d::Transform getPose(timeval stamp)
	{
		geometry_msgs::msg::TransformStamped tf_msg;
		try
		{
			tf_msg = mTfBuffer->lookupTransform("base_link", "odometry", fromTimeval(stamp), 50ms);
		}catch (const tf2::TransformException & ex)
		{
			throw slam3d::InvalidPose(ex.what());
		}
		Eigen::Vector3d trans(tf_msg.transform.translation.x,
		                      tf_msg.transform.translation.y,
		                      tf_msg.transform.translation.z);
		Eigen::Quaterniond quat(tf_msg.transform.rotation.w,
		                        tf_msg.transform.rotation.x,
		                        tf_msg.transform.rotation.y,
		                        tf_msg.transform.rotation.z);
		slam3d::Transform tf(quat);
		tf.translation() = trans;
		return tf;
	}
	
	void handleNewVertex(slam3d::IdType vertex)
	{
		timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
		slam3d::Transform currentPose = getPose(stamp);
		
		if(mLastVertex > 0)
		{
			slam3d::Transform t = mLastOdometricPose.inverse() * currentPose;
			slam3d::SE3Constraint::Ptr se3(new slam3d::SE3Constraint(mName, t, slam3d::Covariance<6>::Identity() * mCovarianceScale));
			mGraph->addConstraint(mLastVertex, vertex, se3);
			mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).corrected_pose * t);
		}
		
		mLastVertex = vertex;
		mLastOdometricPose = currentPose;
	}
	
private:
	tf2_ros::Buffer* mTfBuffer;
	slam3d::Transform mLastOdometricPose;
	slam3d::IdType mLastVertex;

};

class PointcloudMapper : public rclcpp::Node
{
public:
	PointcloudMapper() : Node("pointcloud_mapper"), mClock(this), mTfBuffer(this->get_clock()), mTfListener(mTfBuffer)
	{
		mLogger = new slam3d::Logger(mClock);
		mGraph = new slam3d::BoostGraph(mLogger);
		mSolver = new slam3d::G2oSolver(mLogger);
		mPclSensor = new slam3d::PointCloudSensor("Velodyne", mLogger);
		
		mGraph->setSolver(mSolver);
		
		mMapper = new slam3d::Mapper(mGraph, mLogger, slam3d::Transform::Identity());
		mMapper->registerSensor(mPclSensor);
		
		mTfOdom = new TfOdometry(mGraph, mLogger, &mTfBuffer);
		mMapper->registerPoseSensor(mTfOdom);
		
		mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
			"scan", 10, std::bind(&PointcloudMapper::scanCallback, this, std::placeholders::_1));
		
		mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
	}

private:

	void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
	{
		slam3d::PointCloud pc;
		pcl::fromROSMsg(*msg, pc);
	}

	slam3d::Mapper* mMapper;
	slam3d::BoostGraph* mGraph;
	slam3d::G2oSolver* mSolver;
	slam3d::PointCloudSensor* mPclSensor;
	RosClock mClock;
	slam3d::Logger* mLogger;
	TfOdometry* mTfOdom;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapPublisher;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mScanSubscriber;
	
	tf2_ros::Buffer mTfBuffer;
	tf2_ros::TransformListener mTfListener;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointcloudMapper>());
	rclcpp::shutdown();
	return 0;
}
