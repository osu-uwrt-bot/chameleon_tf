#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <chameleon_tf_msgs/srv/set_transform.hpp>
#include <chameleon_tf_msgs/action/model_frame.hpp>

#include <chrono>

#include "chameleon_tf/transform_stats.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class MobileTfPub : public rclcpp::Node
{
  using ModelFrame = chameleon_tf_msgs::action::ModelFrame;
  using GHModelFrame = rclcpp_action::ServerGoalHandle<ModelFrame>;

public:
  MobileTfPub() : Node("mobile_tf")
  {
    /*
    Deal with parameters
    */

    // declare params
    declare_parameter<std::string>("source_frame");
    declare_parameter<std::string>("target_frame");
    // declared order is xyz for trans and rpy for rot
    declare_parameter<std::vector<double>>("initial_translation");
    declare_parameter<std::vector<double>>("initial_rotation");
    // thresholding while observing
    declare_parameter<double>("stddev_threshold", 0.1);

    // retrieve param values
    lastRelationship = geometry_msgs::msg::Transform();

    // now we check the frame ID's
    sourceName = get_parameter("source_frame").as_string();
    destName = get_parameter("target_frame").as_string();

    // configure the initial transformation
    configureInitTF();

    stddevLimit = get_parameter("stddev_threshold").as_double();

    /*
    Create the ROS contexts
    */

    // timer context for sending the transform
    pubTimer = create_wall_timer(2s, std::bind(&MobileTfPub::tfPublishCallback, this));

    // Initialize the transform broadcaster
    tfBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // make the transform listeners
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // Initialize a service server for moving the transform
    setService = create_service<chameleon_tf_msgs::srv::SetTransform>(
        destName + "/set_transform", std::bind(&MobileTfPub::serviceCallback, this, _1, _2));

    // Initialize an action server for moving the transform based on sampling another transform
    modelAction = rclcpp_action::create_server<ModelFrame>(
        this, destName + "/model_tf", std::bind(&MobileTfPub::handleGoal, this, _1, _2),
        std::bind(&MobileTfPub::handleCancel, this, _1), std::bind(&MobileTfPub::handleAccepted, this, _1));
  }

  void configureInitTF()
  {
    // make sure the initial translation array is valid
    auto translation = get_parameter("initial_translation").as_double_array();
    if (translation.size() < 3)
    {
      throw std::runtime_error("Expected to have translation array of length 3");
    }

    // parse the vector
    lastRelationship.translation.x = translation.at(0);
    lastRelationship.translation.y = translation.at(1);
    lastRelationship.translation.z = translation.at(2);

    // make sure the initial rotation array is valid
    auto rotation = get_parameter("initial_rotation").as_double_array();
    if (rotation.size() < 3)
    {
      throw std::runtime_error("Expected to have translation array of length 3");
    }

    double roll = rotation.at(0), pitch = rotation.at(1), yaw = rotation.at(2);

    // convert RPY to quaternion
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    // build the angular quat message
    lastRelationship.rotation = tf2::toMsg(quat);

    RCLCPP_INFO_STREAM(get_logger(), "Initial translation: (" << lastRelationship.translation.x << ", "
                                                              << lastRelationship.translation.y << ", "
                                                              << lastRelationship.translation.z << ")");

    RCLCPP_INFO_STREAM(get_logger(), "Initial rotation: (" << quat.getX() << ", " << quat.getY() << ", "
                                                           << quat.getZ() << ", " << quat.getW() << ")");
    RCLCPP_INFO_STREAM(get_logger(), "Transforming from " << sourceName << " -> " << destName);
  }

  // callback function for the transform publish timer
  void tfPublishCallback()
  {
    auto tfStamped = geometry_msgs::msg::TransformStamped();
    // assign the transform to be current information
    tfStamped.header.stamp = this->get_clock()->now();
    tfStamped.header.frame_id = sourceName.c_str();
    tfStamped.child_frame_id = destName.c_str();
    tfStamped.transform = lastRelationship;

    // Send the transformation
    tfBroadcaster->sendTransform(tfStamped);
  }

  // set service callback function
  void serviceCallback(const chameleon_tf_msgs::srv::SetTransform::Request::SharedPtr req,
                       chameleon_tf_msgs::srv::SetTransform::Response::SharedPtr resp)
  {
    lastRelationship = req->relationship;

    resp->err_msg = "";
    resp->success = true;
  }

  // action handle goal
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const ModelFrame::Goal> goal)
  {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Received request to watch transform " << goal->monitor_parent << " -> " << goal->monitor_child);
    (void)uuid;

    // see if we are already executing
    if (sampleCount >= 0)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // action handle cancel
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GHModelFrame> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel modeling");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // action handle accept
  void handleAccepted(const std::shared_ptr<GHModelFrame> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MobileTfPub::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GHModelFrame> goal_handle)
  {
    rclcpp::Rate loopRate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ModelFrame::Feedback>();
    auto result = std::make_shared<ModelFrame::Result>();

    // set sample count to zero to indicate start
    sampleCount = 0;

    // make a vector of transforms
    std::vector<geometry_msgs::msg::Transform> transforms;

    // confirm connected TF tree
    // check the trasform exists and is connected
    geometry_msgs::msg::TransformStamped t;
    try
    {
      t = tfBuffer->lookupTransform(goal->monitor_parent, goal->monitor_child, get_clock()->now());
      transforms.push_back(t.transform);
    }
    catch (const tf2::TransformException &ex)
    {
      // if we cant lookup, abort
      result->err_msg = std::string("TF Lookup failed ") + ex.what();
      result->success = false;

      goal_handle->abort(result);

      sampleCount = -1;
      return;
    }

    int iterations = 0;

    // take a number of samples
    while (sampleCount < goal->samples)
    {
      // lookup the current transform
      try
      {
        t = tfBuffer->lookupTransform(goal->monitor_parent, goal->monitor_child, get_clock()->now());
        transforms.push_back(t.transform);

        // increment the counter for success
        sampleCount = transforms.size();
      }
      catch (const tf2::TransformException &ex)
      {
      }

      // check to see if we have done more than 2x the iterations we should have
      if (iterations > 2 * goal->samples)
      {
        // something isnt working, so we abort
        result->err_msg = "Unable to get required samples during expected duration";
        result->success = false;

        sampleCount = -1;

        goal_handle->abort(result);

        return;
      }

      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Cancelling observations");
        return;
      }

      feedback->sample_count = sampleCount;
      goal_handle->publish_feedback(feedback);

      iterations++;

      // wait a bit
      loopRate.sleep();
    }

    // now we need to figure out how to average the transforms
    RCLCPP_INFO(this->get_logger(), "Computing statistics");
    auto stats = tf_stats::getTransformStats(transforms);

    // if the stddev is too high, abort -- this is the final abort metric
    // first check euclidean distance on translation
    double distErr = tf_stats::euclideanDist(stats.stddev.translation);
    if (distErr > stddevLimit)
    {
      // too high, abort
      result->err_msg = "Translation deviation above threshold " + std::to_string(distErr);
      result->success = false;

      sampleCount = -1;

      goal_handle->abort(result);
      return;
    }

    // now check rotation, because of quaternion, this may be funky
    double rotErr = tf_stats::quatDistance(stats.stddev.rotation);
    // TODO Remove this * 10 whe i come up with an accurate err calc
    if (rotErr > stddevLimit * 10)
    {
      // too high, abort
      result->err_msg = "Rotation deviation above threshold " + std::to_string(rotErr);
      result->success = false;

      sampleCount = -1;

      goal_handle->abort(result);
      return;
    }

    // if still okay, update our transform and succeed
    lastRelationship = stats.avg;

    RCLCPP_INFO_STREAM(get_logger(), "Assumed translation: (" << lastRelationship.translation.x << ", "
                                                              << lastRelationship.translation.y << ", " 
                                                              << lastRelationship.translation.z << ")");

    RCLCPP_INFO_STREAM(get_logger(), "Assumed rotation: (" << lastRelationship.rotation.x << ", " 
                                                           << lastRelationship.rotation.y << ", "
                                                           << lastRelationship.rotation.z << ", " 
                                                           << lastRelationship.rotation.w << ")");

    
    result->err_msg = "";
    result->success = true;

    sampleCount = -1;

    goal_handle->succeed(result);
  }

protected:
  geometry_msgs::msg::Transform lastRelationship;

  std::string sourceName, destName;

  double stddevLimit = 0.0;

  // set to this when the action server is not running
  int sampleCount = -1;

  // make the TF listener contexts
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;

  // ROS contexts
  rclcpp::TimerBase::SharedPtr pubTimer;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;
  rclcpp::Service<chameleon_tf_msgs::srv::SetTransform>::SharedPtr setService;
  rclcpp_action::Server<ModelFrame>::SharedPtr modelAction;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MobileTfPub>());

  rclcpp::shutdown();

  return 0;
}
