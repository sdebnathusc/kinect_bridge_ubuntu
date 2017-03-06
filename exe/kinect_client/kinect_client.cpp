#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketStream.h>

#include <atomics/binary_stream.h>

#include <messages/kinect_messages.h>
#include <messages/binary_codec.h>
#include <messages/message_coder.h>

#include <messages/input_tcp_device.h>
#include <messages/png_image_message.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <kinect_bridge2/KinectSpeech.h>
#include <kinect_bridge2/KinectBodies.h>
#include <kinect_bridge2/KinectFaces.h>
#include <kinect_bridge2/KinectHDFaces.h>
#include <kinect_bridge2/KinectDepthImage.h>
#include <kinect_bridge2/KinectColorImage.h>
#include <kinect_bridge2/KinectInfraredImage.h>


class KinectBridge2Client
{
public:
    typedef kinect_bridge2::KinectSpeech _KinectSpeechMsg;
    typedef kinect_bridge2::KinectSpeechPhrase _KinectSpeechPhraseMsg;

    typedef kinect_bridge2::KinectBodies _KinectBodiesMsg;
    typedef kinect_bridge2::KinectBody _KinectBodyMsg;
    typedef kinect_bridge2::KinectJoint _KinectJointMsg;

    typedef kinect_bridge2::KinectFaces _KinectFacesMsg;
    typedef kinect_bridge2::KinectFace _KinectFaceMsg;

    typedef kinect_bridge2::KinectHDFaces _KinectHDFacesMsg;
    typedef kinect_bridge2::KinectHDFace _KinectHDFaceMsg;

    typedef kinect_bridge2::KinectDepthImage _KinectDepthMsg;
    typedef kinect_bridge2::KinectDepthImageInfo _KinectDepthInfoMsg;

    typedef KinectDepthImageMessage<ImageMessage<> > _DepthImageMsg;
    typedef std::shared_ptr<_DepthImageMsg> _DepthImageMsgPtr;
    
    typedef KinectColorImageMessage<ImageMessage<> > _ColorImageMsg;
    typedef std::shared_ptr<_ColorImageMsg> _ColorImageMsgPtr;

    typedef KinectInfraredImageMessage<ImageMessage<> > _InfraredImageMsg;
    typedef std::shared_ptr<_InfraredImageMsg> _InfraredImageMsgPtr;

    ros::NodeHandle nh_rel_;

    ros::Publisher kinect_speech_pub_;
    ros::Publisher kinect_bodies_pub_;
    ros::Publisher kinect_faces_pub_;
    ros::Publisher kinect_hd_faces_pub_;
    ros::Publisher kinect_depth_pub_;
    ros::Publisher kinect_color_pub_;
    ros::Publisher kinect_infrared_pub_;

    InputTCPDevice kinect_bridge_client_;

    uint32_t message_count_;

    MessageCoder<BinaryCodec<> > binary_message_coder_;

    tf::TransformBroadcaster transform_broadcaster_;

    KinectBridge2Client( ros::NodeHandle & nh_rel )
    :
        nh_rel_( nh_rel ),
        kinect_speech_pub_( nh_rel_.advertise<_KinectSpeechMsg>( "speech", 10 ) ),
        kinect_bodies_pub_( nh_rel_.advertise<_KinectBodiesMsg>( "bodies", 10 ) ),
        kinect_faces_pub_( nh_rel_.advertise<_KinectFacesMsg>( "faces", 10 ) ),
        kinect_hd_faces_pub_( nh_rel_.advertise<_KinectHDFacesMsg>( "hdfaces", 10 ) ),
        kinect_depth_pub_( nh_rel_.advertise<sensor_msgs::Image>( "depth", 10 ) ),
	kinect_color_pub_( nh_rel_.advertise<sensor_msgs::Image>( "color", 10 ) ),
	kinect_infrared_pub_( nh_rel_.advertise<sensor_msgs::Image>( "infrared", 10 ) ),
        kinect_bridge_client_( getParam<std::string>( nh_rel_, "server_ip", "localhost" ), getParam<int>( nh_rel_, "server_port", 5903 ) ),
        message_count_( 0 )
    {
        //
    }

    template<class __Data>
    static __Data getParam( ros::NodeHandle & nh, std::string const & param_name, __Data const & default_value )
    {
        __Data result;
        if( nh.getParam( param_name, result ) ) return result;
        return default_value;
    }

    void spin()
    {
        auto last_update = std::chrono::high_resolution_clock::now();
        while( ros::ok() )
        {
            auto now = std::chrono::high_resolution_clock::now();

            if( std::chrono::duration_cast<std::chrono::milliseconds>( now - last_update ).count() >= 1000 )
            {
                last_update = now;
                std::cout << "processed " << message_count_ << " messages" << std::endl;
            }

            try
            {
                if( !kinect_bridge_client_.input_socket_.impl()->initialized() )
                {
                    std::cout << "no server connection" << std::endl;
                    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                    kinect_bridge_client_.openInput();
                    continue;
                }

                CodedMessage<> binary_coded_message;
                kinect_bridge_client_.pull( binary_coded_message );
                processKinectMessage( binary_coded_message );
                message_count_ ++;
 //               std::cout << "message processed" << std::endl;
            }
            catch( messages::MessageException & e )
            {
                std::cout << e.what() << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }
            catch( std::exception & e )
            {
                std::cout << e.what() << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }

            ros::spinOnce();
        }
    }

    void processKinectMessage( CodedMessage<> & coded_message )
    {
        auto & coded_header = coded_message.header_;
//        std::cout << "processing message type: " << coded_message.header_.payload_type_ << std::endl;

        if( coded_header.payload_id_ == KinectSpeechMessage::ID() )
        {
            auto kinect_speech_message = binary_message_coder_.decode<KinectSpeechMessage>( coded_message );

            auto & header = kinect_speech_message.header_;
            auto & payload = kinect_speech_message.payload_;

            _KinectSpeechMsg ros_kinect_speech_message;

            for( size_t i = 0; i < payload.size(); ++i )
            {
                _KinectSpeechPhraseMsg ros_kinect_speech_phrase_message;
                ros_kinect_speech_phrase_message.tag = payload[i].tag_;
                ros_kinect_speech_phrase_message.confidence = payload[i].confidence_;
                ros_kinect_speech_message.phrases.emplace_back( std::move( ros_kinect_speech_phrase_message ) );
            }

            kinect_speech_pub_.publish( ros_kinect_speech_message );
        }        
        else if( coded_header.payload_id_ == _DepthImageMsg::ID() )
        {
            auto kinect_depth_message = binary_message_coder_.decode<KinectDepthImageMessage<>>( coded_message );

            auto & header = kinect_depth_message.header_;
            auto & payload = kinect_depth_message.payload_;

            sensor_msgs::Image ros_kinect_depth_message;
             
            // for( size_t i = 0; i < payload.size_; ++i )
            // {
            //     _KinectDepthInfoMsg ros_kinect_depth_info_message;
            //     ros_kinect_depth_info_message.min_reliable_distance_ = payload[i].min_reliable_distance_;
            //     ros_kinect_depth_info_message.max_reliable_distance_ = payload[i].max_reliable_distance_;
                
            //     ros_kinect_depth_message.depthInfo = ros_kinect_depth_info_message;                
            // }

            ros_kinect_depth_message.height = header.height_;
            ros_kinect_depth_message.width = header.width_;
            ros_kinect_depth_message.encoding = "mono16";
            ros_kinect_depth_message.step = header.width_ * header.pixel_depth_ / 8; 
            ros_kinect_depth_message.is_bigendian = false;

            ros_kinect_depth_message.data.resize(payload.size_); 
            std::memcpy(ros_kinect_depth_message.data.data(), payload.data_, payload.size_);

            // std::cout << "Encoding: " << header.encoding_ << std::endl;
            // std::cout << "Height: " << header.height_ << std::endl;
            // std::cout << "Width: " << header.width_ << std::endl;
            // std::cout << "Payload Size: " << payload.size_ << std::endl;

            kinect_depth_pub_.publish( ros_kinect_depth_message );
        }
	else if( coded_header.payload_id_ == _ColorImageMsg::ID() )
        {
            auto kinect_color_message = binary_message_coder_.decode<KinectColorImageMessage<>>( coded_message );

            auto & header = kinect_color_message.header_;
            auto & payload = kinect_color_message.payload_;

            sensor_msgs::Image ros_kinect_color_message;
             
            ros_kinect_color_message.height = header.height_;
            ros_kinect_color_message.width = header.width_;
            ros_kinect_color_message.encoding = "rgb8";
            ros_kinect_color_message.step = header.width_ * header.num_channels_ * header.pixel_depth_ / 8; 
            ros_kinect_color_message.is_bigendian = false;

            ros_kinect_color_message.data.resize(payload.size_); 
            std::memcpy(ros_kinect_color_message.data.data(), payload.data_, payload.size_);

            //std::cout << "Encoding: " << static_cast<string>(header.encoding_) << std::endl;
            //std::cout << "Height: " << header.height_ << std::endl;
            //std::cout << "Width: " << header.width_ << std::endl;
            //std::cout << "Channels: " << static_cast<int>(header.num_channels_) << std::endl;
            //std::cout << "Payload Size: " << payload.size_ << std::endl;

            kinect_color_pub_.publish( ros_kinect_color_message );
        }
	else if( coded_header.payload_id_ == _InfraredImageMsg::ID() )
        {
            auto kinect_infrared_message = binary_message_coder_.decode<KinectInfraredImageMessage<>>( coded_message );

            auto & header = kinect_infrared_message.header_;
            auto & payload = kinect_infrared_message.payload_;

            sensor_msgs::Image ros_kinect_infrared_message;
             
            ros_kinect_infrared_message.height = header.height_;
            ros_kinect_infrared_message.width = header.width_;
            ros_kinect_infrared_message.encoding = "mono16";
            ros_kinect_infrared_message.step = header.width_ * header.pixel_depth_ / 8; 
            ros_kinect_infrared_message.is_bigendian = false;

            ros_kinect_infrared_message.data.resize(payload.size_); 
            std::memcpy(ros_kinect_infrared_message.data.data(), payload.data_, payload.size_);

            //std::cout << "Encoding: " << static_cast<std::string>(header.encoding_) << std::endl;
            //std::cout << "Height: " << header.height_ << std::endl;
            //std::cout << "Width: " << header.width_ << std::endl;
            //std::cout << "Payload Size: " << payload.size_ << std::endl;

            kinect_infrared_pub_.publish( ros_kinect_infrared_message );
        }
        else if( coded_header.payload_id_ == KinectBodiesMessage::ID() )
        {
            auto bodies_msg = binary_message_coder_.decode<KinectBodiesMessage>( coded_message );

            auto const & header = bodies_msg.header_;
            auto const & payload = bodies_msg.payload_;

	    //std::cout << "I am in : " << payload.size() << std::endl;

            _KinectBodiesMsg ros_bodies_msg;

            // get map of KinectJointMessage::JointType -> human-readable name
            auto const & joint_names_map = KinectJointMessage::getJointNamesMap();

            // for each body message
            for( size_t body_idx = 0; body_idx < payload.size(); ++body_idx )
            {
                _KinectBodyMsg ros_body_msg;
                auto const & body_msg = payload[body_idx];

                ros_body_msg.is_tracked = body_msg.is_tracked_;
                ros_body_msg.hand_state_left = static_cast<uint8_t>( body_msg.hand_state_left_ );
                ros_body_msg.hand_state_right = static_cast<uint8_t>( body_msg.hand_state_right_ );

                auto const & joints_msg = body_msg.joints_;
                auto & ros_joints_msg = ros_body_msg.joints;

                std::stringstream tf_frame_basename_ss;
                tf_frame_basename_ss << "/kinect_client/skeleton" << body_idx << "/";

                // for each joint message
                for( size_t joint_idx = 0; joint_idx < joints_msg.size(); ++joint_idx )
                {
                    auto const & joint_msg = joints_msg[joint_idx];
                    _KinectJointMsg ros_joint_msg;

                    ros_joint_msg.joint_type = static_cast<uint8_t>( joint_msg.joint_type_ );
                    ros_joint_msg.tracking_state = static_cast<uint8_t>( joint_msg.tracking_state_ );

		    //std::cout << "Dummy1: " << static_cast<uint8_t>(joint_msg.position_.x) << std::endl;
		    //ros_joint_msg.dummy_val = static_cast<uint8_t>( joint_msg.dummy_val_ );

		    ros_joint_msg.tracker.x = joint_msg.tracker_.x;
		    ros_joint_msg.tracker.y = joint_msg.tracker_.y;

                    ros_joint_msg.position.x = joint_msg.position_.x;
                    ros_joint_msg.position.y = joint_msg.position_.y;
                    ros_joint_msg.position.z = joint_msg.position_.z;

                    ros_joint_msg.orientation.x = joint_msg.orientation_.x;
                    ros_joint_msg.orientation.y = joint_msg.orientation_.y;
                    ros_joint_msg.orientation.z = joint_msg.orientation_.z;
                    ros_joint_msg.orientation.w = joint_msg.orientation_.w;

                    ros_joints_msg.emplace_back( std::move( ros_joint_msg ) );

                    tf::Transform joint_transform
                    (
                        joint_msg.tracking_state_ == KinectJointMessage::TrackingState::TRACKED ? tf::Quaternion( joint_msg.orientation_.x, joint_msg.orientation_.y, joint_msg.orientation_.z, joint_msg.orientation_.w ).normalized() : tf::Quaternion( 0, 0, 0, 1 ),
                        tf::Vector3( joint_msg.position_.x, joint_msg.position_.y, joint_msg.position_.z )
                    );

                    // if the rotation is nan, zero it out to make TF happy
                    if( std::isnan( joint_transform.getRotation().getAngle() ) ) joint_transform.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );

		    //std::cout << "Rotation x: " << tf_message_callback_queue_.rotation.x << std::endl;
                    static tf::Transform const trunk_norm_rotation_tf( tf::Quaternion( -M_PI_2, -M_PI_2, 0 ).normalized() );

                    switch( joint_msg.joint_type_ )
                    {
                    case KinectJointMessage::JointType::SPINE_BASE:
                    case KinectJointMessage::JointType::SPINE_MID:
                    case KinectJointMessage::JointType::NECK:
                    case KinectJointMessage::JointType::HEAD:
                    case KinectJointMessage::JointType::SPINE_SHOULDER:
                        joint_transform *= trunk_norm_rotation_tf;
                        break;
                    default:
                        break;
                    }

                    transform_broadcaster_.sendTransform( tf::StampedTransform( joint_transform, ros::Time::now(), "/kinect", tf_frame_basename_ss.str() + joint_names_map.find(joint_msg.joint_type_)->second ) );
                }
                ros_bodies_msg.bodies.emplace_back( std::move( ros_body_msg ) );
            }
            kinect_bodies_pub_.publish( ros_bodies_msg );
        }
	else if( coded_header.payload_id_ == KinectFacesMessage::ID() )
        {
            auto faces_msg = binary_message_coder_.decode<KinectFacesMessage>( coded_message );

            auto const & header = faces_msg.header_;
            auto const & payload = faces_msg.payload_;

	    std::cout << "I am in Face	: " << payload.size() << std::endl;

            _KinectFacesMsg ros_faces_msg;

            
            // for each body message
            for( size_t face_idx = 0; face_idx < payload.size(); ++face_idx )
            {
                _KinectFaceMsg ros_face_msg;
                auto const & face_msg = payload[face_idx];

                ros_face_msg.is_tracked = face_msg.is_tracked_;
		ros_face_msg.Pitch = face_msg.pitch_;
		ros_face_msg.Yaw = face_msg.yaw_;
		ros_face_msg.Roll = face_msg.roll_;
                ros_face_msg.Happy = face_msg.Happy_;
                ros_face_msg.Engaged = face_msg.Engaged_;
                ros_face_msg.WearingGlasses = face_msg.WearingGlasses_;
                ros_face_msg.LeftEyeClosed = face_msg.LeftEyeClosed_;
                ros_face_msg.RightEyeClosed = face_msg.RightEyeClosed_;
                ros_face_msg.MouthOpen = face_msg.MouthOpen_;
                ros_face_msg.MouthMoved = face_msg.MouthMoved_;
                ros_face_msg.LookingAway = face_msg.LookingAway_;

		ros_face_msg.BoundingBox[0] = face_msg.boundingBox_.x; // Top
		ros_face_msg.BoundingBox[1] = face_msg.boundingBox_.y; // Right
		ros_face_msg.BoundingBox[2] = face_msg.boundingBox_.z; // Bottom
		ros_face_msg.BoundingBox[3] = face_msg.boundingBox_.w; // Left
	
		ros_face_msg.EyeLeft[0] = face_msg.EyeLeft_.x; // X
		ros_face_msg.EyeLeft[1] = face_msg.EyeLeft_.y; // Y

		ros_face_msg.EyeRight[0] = face_msg.EyeRight_.x;
		ros_face_msg.EyeRight[1] = face_msg.EyeRight_.y;

		ros_face_msg.Nose[0] = face_msg.Nose_.x;
		ros_face_msg.Nose[1] = face_msg.Nose_.y;

		ros_face_msg.MouthCornerLeft[0] = face_msg.MouthCornerLeft_.x;
		ros_face_msg.MouthCornerLeft[1] = face_msg.MouthCornerLeft_.y;

		ros_face_msg.MouthCornerRight[0] = face_msg.MouthCornerRight_.x;
		ros_face_msg.MouthCornerRight[1] = face_msg.MouthCornerRight_.y;
                                
                ros_faces_msg.faces.emplace_back( std::move( ros_face_msg ) );
            }
            kinect_faces_pub_.publish( ros_faces_msg );
        }
	else if( coded_header.payload_id_ == KinectHDFacesMessage::ID() )
        {
            auto hd_faces_msg = binary_message_coder_.decode<KinectHDFacesMessage>( coded_message );

            auto const & header = hd_faces_msg.header_;
            auto const & payload = hd_faces_msg.payload_;

	    std::cout << "I am in HD Face	: " << payload.size() << std::endl;

            _KinectHDFacesMsg ros_hd_faces_msg;

            
            // for each body message
            for( size_t hd_face_idx = 0; hd_face_idx < payload.size(); ++hd_face_idx )
            {
                _KinectHDFaceMsg ros_hd_face_msg;
                auto const & hd_face_msg = payload[hd_face_idx];

                ros_hd_face_msg.is_tracked = hd_face_msg.is_tracked_;
                                
                ros_hd_faces_msg.hdfaces.emplace_back( std::move( ros_hd_face_msg ) );
            }
            kinect_hd_faces_pub_.publish( ros_hd_faces_msg );
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "kinect_client" );
    ros::NodeHandle nh_rel( "~" );

    KinectBridge2Client kinect_bridge2_client( nh_rel );

    kinect_bridge2_client.spin();

    return 0;
}
