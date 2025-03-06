#include "../include/t265_realsense_node.h"
#include <fstream>
#include <cstring>

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(rclcpp::Node& node,
                                     rs2::device dev, std::shared_ptr<Parameters> parameters) : 
                                     BaseRealSenseNode(node, dev, parameters, false),
                                     _wo_snr(dev.first<rs2::wheel_odometer>()),
                                     _use_odom_in(false) 
                                     {
                                         _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
                                         initializeOdometryInput();
                                     }

void T265RealsenseNode::initializeOdometryInput()
{
    std::string calib_odom_file;
    std::string param_name = std::string("calib_odom_file");

    calib_odom_file = _parameters->setParam<std::string>(param_name, std::string(""));

    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        return;
    }
    std::ifstream calibrationFile(calib_odom_file);
    if (!calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
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

    std::string topic_odom_in;
    std::string param_name = std::string("topic_odom_in");
    topic_odom_in = _parameters->setParam<std::string>(param_name, DEFAULT_TOPIC_ODOM_IN);
    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

    _odom_subscriber = _node.create_subscription<nav_msgs::msg::Odometry>(topic_odom_in, 1, std::bind(&T265RealsenseNode::odom_in_callback, this, std::placeholders::_1));
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};

    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}

void T265RealsenseNode::calcAndAppendTransformMsgs(const rs2::stream_profile& profile,
                                                   const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());

    // rotation quaternion from ROS CS to Optical CS
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0, -M_PI/2);  // R,P,Y rotations over the original axes

    // zero rotation quaternion, used for IMU
    tf2::Quaternion zero_rot_quaternions;
    zero_rot_quaternions.setRPY(0, 0, 0);

    // zero translation, used for moving from ROS frame to its correspending Optical frame
    float3 zero_trans{0, 0, 0};

    rclcpp::Time transform_ts_ = _node.now();

    // extrinsic from A to B is the position of A relative to B
    // TF from A to B is the transformation to be done on A to get to B
    // so, we need to calculate extrinsics in two opposite ways, one for extrinsic topic
    // and the second is for transformation topic (TF)
    rs2_extrinsics normal_ex;  // used to for extrinsics topic
    rs2_extrinsics tf_ex; // used for TF

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

    // publish normal extrinsics e.g. /camera/extrinsics/depth_to_color
    publishExtrinsicsTopic(sip, normal_ex);

    // Representing Rotations with Quaternions
    // see https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations for reference

    // Q defines a quaternion rotation from <base profile CS> to <current profile CS>
    // for example, rotation from depth to infra2
    auto Q = rotationMatrixToQuaternion(tf_ex.rotation);

    float3 trans{tf_ex.translation[0], tf_ex.translation[1], tf_ex.translation[2]};

    // Rotation order is important (start from left to right):
    // 1. quaternion_optical.inverse() [ROS -> Optical]
    // 2. Q [Optical -> Optical] (usually no rotation, but might be very small rotations between sensors, like from Depth to Color)
    // 3. quaternion_optical [Optical -> ROS]
    // We do all these products since we want to finish in ROS CS, while Q is a rotation from optical to optical,
    // and cant be used directly in ROS TF without this combination
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    if (sip == POSE)
    {
        Q = Q.inverse();
	    append_static_tf_msg(transform_ts_, trans, Q, FRAME_ID(sip), _base_frame_id);
    }
    else
    {
        append_static_tf_msg(transform_ts_, trans, Q, _base_frame_id, FRAME_ID(sip));
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
        // Add align_depth_to if exist:
        if (profile.is<rs2::video_stream_profile>() &&
                profile.stream_type() != RS2_STREAM_DEPTH &&
                profile.stream_index() == 1)
        {
            append_static_tf_msg(transform_ts_, trans, Q, _base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
            append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
        }
    }
}

