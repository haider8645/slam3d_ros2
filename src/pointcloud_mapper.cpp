#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/core/Mapper.hpp>

using namespace std::chrono_literals;

#define TF_TIMEOUT 50ms

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
	TfOdometry(slam3d::Graph* g, slam3d::Logger* l, tf2_ros::Buffer* tf, const std::string& robot_f, const std::string& odom_f)
	: slam3d::PoseSensor("Odometry", g, l), mTfBuffer(tf), mRobotFrame(robot_f), mOdometryFrame(odom_f) {}
	
	slam3d::Transform getPose(timeval stamp)
	{
		try
		{
			return tf2::transformToEigen(mTfBuffer->lookupTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), TF_TIMEOUT));
		}catch (const tf2::TransformException & ex)
		{
			throw slam3d::InvalidPose(ex.what());
		}
	}
	
	void handleNewVertex(slam3d::IdType vertex)
	{
		timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
		slam3d::Transform currentPose = getPose(stamp);
		
		if(mLastVertex > 0)
		{
			slam3d::Transform t = mLastOdometricPose.inverse() * currentPose;
			slam3d::SE3Constraint::Ptr se3(new slam3d::SE3Constraint(mName, t, slam3d::Covariance<6>::Identity() * 0.01));
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
	std::string mRobotFrame;
	std::string mOdometryFrame;

};

class PointcloudMapper : public rclcpp::Node
{
public:
	PointcloudMapper() : Node("pointcloud_mapper"), mClock(this), mTfBuffer(this->get_clock()),
		mTfListener(mTfBuffer), mTfBroadcaster(this)
	{
		mLogger = new slam3d::Logger(mClock);
		mLogger->setLogLevel(slam3d::DEBUG);
		
		mGraph = new slam3d::BoostGraph(mLogger);
		mSolver = new slam3d::G2oSolver(mLogger);
		mPclSensor = new slam3d::PointCloudSensor("Velodyne", mLogger);
		
		slam3d::RegistrationParameters regParams;
		regParams.point_cloud_density = 1.0;
		regParams.maximum_iterations = 10;
		regParams.max_correspondence_distance = 2.0;
		
		mPclSensor->setMinPoseDistance(0.5, 1.0);
		mPclSensor->setMapResolution(0.2);
		mPclSensor->setRegistrationParameters(regParams, false);
		mPclSensor->setNeighborRadius(5.0, 1);
		mPclSensor->setLinkPrevious(true);
		
		mMapFrame = "map";
		mOdometryFrame = "odom";
		mRobotFrame = "husky";
		mLaserFrame = "husky/base_link/front_laser";
		
		mGraph->setSolver(mSolver);
		
		mMapper = new slam3d::Mapper(mGraph, mLogger, slam3d::Transform::Identity());
		mMapper->registerSensor(mPclSensor);
		
		mTfOdom = new TfOdometry(mGraph, mLogger, &mTfBuffer, mRobotFrame, mOdometryFrame);
		mMapper->registerPoseSensor(mTfOdom);
		
		mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10,
			std::bind(&PointcloudMapper::scanCallback, this, std::placeholders::_1));
		
		mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
		
		mGenerateMapService = create_service<std_srvs::srv::Empty>("generate_map",
			std::bind(&PointcloudMapper::generateMap, this, std::placeholders::_1, std::placeholders::_2));
	}

private:

	void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		try
		{
			slam3d::PointCloud::Ptr pc(new slam3d::PointCloud);
			pcl::fromROSMsg(*msg, *pc);
			
			slam3d::Transform laser_pose = tf2::transformToEigen(
				mTfBuffer.lookupTransform(mRobotFrame, mLaserFrame, msg->header.stamp, TF_TIMEOUT));
			
			slam3d::Transform odometry_pose = tf2::transformToEigen(
				mTfBuffer.lookupTransform(mOdometryFrame, mRobotFrame, msg->header.stamp, TF_TIMEOUT));

			slam3d::PointCloud::Ptr scan = mPclSensor->downsample(pc, 0.1);
			
			slam3d::PointCloudMeasurement::Ptr m(new slam3d::PointCloudMeasurement(scan, "Robot", mPclSensor->getName(), laser_pose));
			
			if(mPclSensor->addMeasurement(m, odometry_pose))
			{
				// Publish "map" -> "odometry"
				slam3d::Transform drift = mMapper->getCurrentPose() * odometry_pose.inverse();
				mDrift = tf2::eigenToTransform(drift);
				mDrift.header.frame_id = mMapFrame;
				mDrift.child_frame_id = mOdometryFrame;
			}
			mLastScanTime = msg->header.stamp;
			mDrift.header.stamp = mLastScanTime;
			mTfBroadcaster.sendTransform(mDrift);
		}
		catch(std::exception& e)
		{
			mLogger->message(slam3d::ERROR, e.what());
		}
	}

	void generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	                       std::shared_ptr<std_srvs::srv::Empty::Response> response)
	{
		mGraph->optimize();
		slam3d::VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
		slam3d::PointCloud::Ptr map = mPclSensor->buildMap(vertices);
		sensor_msgs::msg::PointCloud2 pc2_msg;
		pcl::toROSMsg(*map, pc2_msg);
		pc2_msg.header.frame_id = mMapFrame;
		pc2_msg.header.stamp = mLastScanTime;
		mMapPublisher->publish(pc2_msg);
	}

	slam3d::Mapper* mMapper;
	slam3d::BoostGraph* mGraph;
	slam3d::G2oSolver* mSolver;
	slam3d::PointCloudSensor* mPclSensor;
	RosClock mClock;
	slam3d::Logger* mLogger;
	TfOdometry* mTfOdom;
	
	std::string mMapFrame;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	std::string mLaserFrame;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapPublisher;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mScanSubscriber;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mGenerateMapService;
	
	tf2_ros::Buffer mTfBuffer;
	tf2_ros::TransformListener mTfListener;
	tf2_ros::TransformBroadcaster mTfBroadcaster;
	rclcpp::Time mLastScanTime;
	geometry_msgs::msg::TransformStamped mDrift;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointcloudMapper>());
	rclcpp::shutdown();
	return 0;
}
