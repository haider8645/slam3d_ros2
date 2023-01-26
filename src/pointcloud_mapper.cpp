#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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


class PointcloudMapper : public rclcpp::Node
{
public:
	PointcloudMapper() : Node("pointcloud_mapper"), mClock(this)
	{
		mLogger = new slam3d::Logger(mClock);
		mGraph = new slam3d::BoostGraph(mLogger);
		mSolver = new slam3d::G2oSolver(mLogger);
		mPclSensor = new slam3d::PointCloudSensor("Velodyne", mLogger);
		
		mGraph->setSolver(mSolver);
		
		mMapper = new slam3d::Mapper(mGraph, mLogger, slam3d::Transform::Identity());
		mMapper->registerSensor(mPclSensor);
		
	}

private:

	slam3d::Mapper* mMapper;
	slam3d::BoostGraph* mGraph;
	slam3d::G2oSolver* mSolver;
	slam3d::PointCloudSensor* mPclSensor;
	RosClock mClock;
	slam3d::Logger* mLogger;
	
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointcloudMapper>());
	rclcpp::shutdown();
	return 0;
}
