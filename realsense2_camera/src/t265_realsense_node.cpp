#include "../include/t265_realsense_node.h"
#include <fstream>
#include <cstring>
#include <atomic>
#include <mutex>

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(rclcpp::Node& node,
                                     rs2::device dev,
                                     std::shared_ptr<Parameters> parameters)
    : BaseRealSenseNode(node, dev, parameters, false)
    , _use_odom_in(false)
{
    try {
        _wo_snr.emplace(dev.first<rs2::wheel_odometer>());
    } catch (...) {
        _wo_snr.reset();
    }

    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
    initializeOdometryInput();
}

void T265RealsenseNode::initializeOdometryInput()
{
    const std::string param_name = "calib_odom_file";
    const auto calib_odom_file = _parameters->setParam<std::string>(param_name, std::string(""));

    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        _use_odom_in = false;
        return;
    }

    std::ifstream calibrationFile(calib_odom_file);
    if (!calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found");
    }

    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
                               std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_wo_snr.has_value())
    {
        ROS_WARN("Wheel odometer interface not available; disabling input odometry.");
        _use_odom_in = false;
        return;
    }

    std::lock_guard<std::mutex> lk(_rs_mutex);

    if (!_node_alive.load(std::memory_order_acquire))
    {
        _use_odom_in = false;
        return;
    }

    if (!_wo_snr->load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file");
    }

    _use_odom_in = true;
}

void T265RealsenseNode::publishTopics()
{
    BaseRealSenseNode::publishTopics();
    setupSubscribers();
}

void T265RealsenseNode::setupSubscribers()
{
    if (!_use_odom_in) return;

    const std::string param_name = "topic_odom_in";
    const auto topic_odom_in = _parameters->setParam<std::string>(param_name, DEFAULT_TOPIC_ODOM_IN);

    auto qos_realtime = rclcpp::QoS(rclcpp::KeepLast(20))
                            .reliable()
                            .deadline(std::chrono::milliseconds(40))
                            .liveliness(rclcpp::LivelinessPolicy::Automatic)
                            .liveliness_lease_duration(std::chrono::seconds(1));

    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

    _odom_subscriber = _node.create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_in, qos_realtime,
        std::bind(&T265RealsenseNode::odom_in_callback, this, std::placeholders::_1));
}

void T265RealsenseNode::onBeforeHardwareReset()
{
    _odom_subscriber.reset();

    std::lock_guard<std::mutex> lk(_rs_mutex);

    _wo_snr.reset();
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!_node_alive.load(std::memory_order_acquire))
        return;

    std::lock_guard<std::mutex> lk(_rs_mutex);
    if (!_node_alive.load(std::memory_order_relaxed))
        return;

    if (!_use_odom_in)
        return;

    if (!_wo_snr.has_value())
        return;

    rclcpp::Time current_time = rclcpp::Clock().now();

    if (!first_callback_)
    {
        double time_diff = (current_time - last_callback_time_).seconds();
        if (time_diff > 0.1)
            ROS_WARN_STREAM("The odom is taking a mad long time, " << time_diff << " seconds");
    }
    else
    {
        first_callback_ = false;
    }
    last_callback_time_ = current_time;

    rs2_vector velocity{
        -(float)(msg->twist.twist.linear.y),
         (float)(msg->twist.twist.linear.z),
        -(float)(msg->twist.twist.linear.x)
    };

    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);

    try
    {
        _wo_snr->send_wheel_odometry(0, 0, velocity);
    }
    catch (const std::exception& e)
    {
        ROS_WARN_STREAM("send_wheel_odometry exception: " << e.what());
    }
}

void T265RealsenseNode::calcAndAppendTransformMsgs(const rs2::stream_profile& profile,
                                                   const rs2::stream_profile& base_profile)
{
    stream_index_pair sip(profile.stream_type(), profile.stream_index());

    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI/2, 0, -M_PI/2);

    tf2::Quaternion zero_rot_quaternions;
    zero_rot_quaternions.setRPY(0, 0, 0);

    float3 zero_trans{0, 0, 0};

    rclcpp::Time transform_ts_ = _node.now();

    rs2_extrinsics normal_ex;
    rs2_extrinsics tf_ex;

    try
    {
        normal_ex = base_profile.get_extrinsics_to(profile);
        tf_ex = profile.get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(base_profile.stream_type()) << ", "
                                << base_profile.stream_index() << ") -> ("
                                << rs2_stream_to_string(profile.stream_type()) << ", "
                                << profile.stream_index() << "): "
                                << e.what()
                                << " : using unity as default.");
            normal_ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
            tf_ex = normal_ex;
        }
        else
        {
            throw e;
        }
    }

    publishExtrinsicsTopic(sip, normal_ex);

    auto Q = rotationMatrixToQuaternion(tf_ex.rotation);

    float3 trans{tf_ex.translation[0], tf_ex.translation[1], tf_ex.translation[2]};

    Q = quaternion_optical * Q * quaternion_optical.inverse();
    if (sip == POSE)
    {
        Q = Q.inverse();
        if (!std::isfinite(Q.x()) ||
            !std::isfinite(Q.y()) ||
            !std::isfinite(Q.z()) ||
            !std::isfinite(Q.w()))
        {
            return;
        }
        append_static_tf_msg(transform_ts_, trans, Q, FRAME_ID(sip), _base_frame_id);
    }
    else
    {
        Q = Q.inverse() * Q;
        if (!std::isfinite(Q.x()) ||
            !std::isfinite(Q.y()) ||
            !std::isfinite(Q.z()) ||
            !std::isfinite(Q.w()))
        {
            return;
        }
        append_static_tf_msg(transform_ts_, trans, Q, _base_frame_id, FRAME_ID(sip));
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }
}
