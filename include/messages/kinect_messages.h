#ifndef _MESSAGES_KINECTMESSAGES_H_
#define _MESSAGES_KINECTMESSAGES_H_

#include <map>

#include <messages/container_messages.h>
#include <messages/utility_messages.h>
#include <messages/image_message.h>
#include <messages/png_image_message.h>
#include <messages/audio_message.h>
#include <messages/geometry_messages.h>

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectColorImageMessage : public MultiMessage<__ImageMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectColorImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectColorImageMessage )
};

// ####################################################################################################
class KinectDepthImageInfoMessage : public SerializableInterface
{
public:
    uint16_t min_reliable_distance_;
    uint16_t max_reliable_distance_;

    // ====================================================================================================
    KinectDepthImageInfoMessage()
    :
        min_reliable_distance_( 0 ),
        max_reliable_distance_( 0 )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << min_reliable_distance_;
        archive << max_reliable_distance_;
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> min_reliable_distance_;
        archive >> max_reliable_distance_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectDepthImageInfoMessage )
};

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectDepthImageMessage : public MultiMessage<__ImageMessage, KinectDepthImageInfoMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, KinectDepthImageInfoMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectDepthImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectDepthImageMessage )
};

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectInfraredImageMessage : public MultiMessage<__ImageMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectInfraredImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectInfraredImageMessage )
};

// ####################################################################################################
class KinectAudioInfoMessage : public SerializableInterface
{
public:
    float beam_angle_;
    float beam_angle_confidence_;

    // ====================================================================================================
    KinectAudioInfoMessage( float beam_angle = 0, float beam_angle_confidence = 0 )
    :
        beam_angle_( beam_angle ),
        beam_angle_confidence_( beam_angle_confidence )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << beam_angle_;
        archive << beam_angle_confidence_;
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> beam_angle_;
        archive >> beam_angle_confidence_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectAudioInfoMessage )
};

// ####################################################################################################
template<class __AudioMessage = AudioMessage<> >
class KinectAudioMessage : public MultiMessage<__AudioMessage, KinectAudioInfoMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__AudioMessage, KinectAudioInfoMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectAudioMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectAudioMessage )
};

// ####################################################################################################
class KinectJointMessage : public SerializableInterface
{
public:
    enum class JointType
    {
        SPINE_BASE = 0,
        SPINE_MID = 1,
        NECK = 2,
        HEAD = 3,
        SHOULDER_LEFT = 4,
        ELBOW_LEFT = 5,
        WRIST_LEFT = 6,
        HAND_LEFT = 7,
        SHOULDER_RIGHT = 8,
        ELBOW_RIGHT = 9,
        WRIST_RIGHT = 10,
        HAND_RIGHT = 11,
        HIP_LEFT = 12,
        KNEE_LEFT = 13,
        ANKLE_LEFT = 14,
        FOOT_LEFT = 15,
        HIP_RIGHT = 16,
        KNEE_RIGHT = 17,
        ANKLE_RIGHT = 18,
        FOOT_RIGHT = 19,
        SPINE_SHOULDER = 20,
        HANDTIP_LEFT = 21,
        THUMB_LEFT = 22,
        HANDTIP_RIGHT = 23,
        THUMB_RIGHT = 24
    };

    enum class TrackingState
    {
        NOT_TRACKED = 0,
        INFERRED = 1,
        TRACKED = 2
    };

    typedef std::map<JointType, std::string> _JointNamesMap;

    static _JointNamesMap buildJointNamesMap()
    {
        _JointNamesMap joint_names_map;
        joint_names_map[JointType::SPINE_BASE] = "spine_base";
        joint_names_map[JointType::SPINE_MID] = "spine_mid";
        joint_names_map[JointType::NECK] = "neck";
        joint_names_map[JointType::HEAD] = "head";
        joint_names_map[JointType::SHOULDER_LEFT] = "shoulder_left";
        joint_names_map[JointType::ELBOW_LEFT] = "elbow_left";
        joint_names_map[JointType::WRIST_LEFT] = "wrist_left";
        joint_names_map[JointType::HAND_LEFT] = "hand_left";
        joint_names_map[JointType::SHOULDER_RIGHT] = "shoulder_right";
        joint_names_map[JointType::ELBOW_RIGHT] = "elbow_right";
        joint_names_map[JointType::WRIST_RIGHT] = "wrist_right";
        joint_names_map[JointType::HAND_RIGHT] = "hand_right";
        joint_names_map[JointType::HIP_LEFT] = "hip_left";
        joint_names_map[JointType::KNEE_LEFT] = "knee_left";
        joint_names_map[JointType::ANKLE_LEFT] = "ankle_left";
        joint_names_map[JointType::FOOT_LEFT] = "foot_left";
        joint_names_map[JointType::HIP_RIGHT] = "hip_right";
        joint_names_map[JointType::KNEE_RIGHT] = "knee_right";
        joint_names_map[JointType::ANKLE_RIGHT] = "ankle_right";
        joint_names_map[JointType::FOOT_RIGHT] = "foot_right";
        joint_names_map[JointType::SPINE_SHOULDER] = "spine_shoulder";
        joint_names_map[JointType::HANDTIP_LEFT] = "handtip_left";
        joint_names_map[JointType::THUMB_LEFT] = "thumb_left";
        joint_names_map[JointType::HANDTIP_RIGHT] = "handtip_right";
        joint_names_map[JointType::THUMB_RIGHT] = "thumb_right";
        return joint_names_map;
    }

    static _JointNamesMap const & getJointNamesMap()
    {
        static _JointNamesMap const & joint_names_map( buildJointNamesMap() );

        return joint_names_map;
    }

    PointMessage<float, 2> tracker_;
    PointMessage<float, 3> position_;
    PointMessage<float, 4> orientation_;

    JointType joint_type_;
    TrackingState tracking_state_;
    //uint8_t dummy_val_;

    // ====================================================================================================
    KinectJointMessage()
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << static_cast<uint8_t>( joint_type_ );
        archive << static_cast<uint8_t>( tracking_state_ );
        //archive << static_cast<uint8_t>( dummy_val_ );
	tracker_.pack( archive );
        position_.pack( archive );
        orientation_.pack( archive );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        uint8_t joint_type;
        archive >> joint_type;
        joint_type_ = static_cast<JointType>( joint_type );

        uint8_t tracking_state;
        archive >> tracking_state;
        tracking_state_ = static_cast<TrackingState>( tracking_state );

        //uint8_t dummy_val;
        //archive >> dummy_val;
        //dummy_val_ = static_cast<uint8_t>( dummy_val );

	tracker_.unpack( archive );
        position_.unpack( archive );
        orientation_.unpack( archive );
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectJointMessage )
};

// ####################################################################################################
class KinectBodyMessage : public VectorMessage<KinectJointMessage>
{
public:
    typedef VectorMessage<KinectJointMessage> _Message;

    enum class HandState
    {
        UNKNOWN = 0,
        NOT_TRACKED = 1,
        OPEN = 2,
        CLOSED = 3,
        LASSO = 4
    };

    std::vector<KinectJointMessage> & joints_;
    uint8_t is_tracked_;
    HandState hand_state_left_;
    HandState hand_state_right_;
    uint64_t tracking_id_;

    // ====================================================================================================
    KinectBodyMessage()
    :
        _Message(),
        joints_( this->payload_ ),
        is_tracked_( 0 ),
        hand_state_left_( HandState::UNKNOWN ),
        hand_state_right_( HandState::UNKNOWN ),
        tracking_id_( 0 )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << is_tracked_;
        archive << static_cast<uint8_t>( hand_state_left_ );
        archive << static_cast<uint8_t>( hand_state_right_ );
        archive << tracking_id_;
        _Message::pack( archive );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> is_tracked_;

        uint8_t hand_state_left;
        archive >> hand_state_left;
        hand_state_left_ = static_cast<HandState>( hand_state_left );

        uint8_t hand_state_right;
        archive >> hand_state_right;
        hand_state_right_ = static_cast<HandState>( hand_state_right );

        archive >> tracking_id_;

        _Message::unpack( archive );
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectBodyMessage )
};

// ####################################################################################################
class KinectBodiesMessage : public MultiMessage<VectorMessage<KinectBodyMessage>, TimeStampMessage>
{
public:
    typedef MultiMessage<VectorMessage<KinectBodyMessage>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectBodiesMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectBodiesMessage )
};


class KinectFaceMessage : public SerializableInterface
{
public:
	
	uint8_t is_tracked_;
        int8_t pitch_;
	int8_t yaw_;
	int8_t roll_;
	std::string Happy_;
	std::string Engaged_;
	std::string WearingGlasses_;
	std::string LeftEyeClosed_;
	std::string RightEyeClosed_;
	std::string MouthOpen_;
	std::string MouthMoved_;
	std::string LookingAway_;
	PointMessage<int, 4> boundingBox_;
	PointMessage<int, 2> EyeLeft_;
	PointMessage<int, 2> EyeRight_;
	PointMessage<int, 2> Nose_;
	PointMessage<int, 2> MouthCornerLeft_;
	PointMessage<int, 2> MouthCornerRight_;
	
	// ====================================================================================================
	KinectFaceMessage()
		:
		is_tracked_(0),
		pitch_(0),
		yaw_(0),
		roll_(0),
		Happy_("NA"),
		Engaged_("NA"),
	   	WearingGlasses_("NA"),
	   	LeftEyeClosed_("NA"),
	   	RightEyeClosed_("NA"),
	   	MouthOpen_("NA"),
	   	MouthMoved_("NA"),
	   	LookingAway_("NA")
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << is_tracked_;
		archive << pitch_;
		archive << yaw_;
		archive << roll_;
		archive << Happy_;
		archive << Engaged_;
		archive << WearingGlasses_;
		archive << LeftEyeClosed_;
		archive << RightEyeClosed_;
		archive << MouthOpen_;
		archive << MouthMoved_;
		archive << LookingAway_;
		boundingBox_.pack(archive);
		EyeLeft_.pack(archive);
		EyeRight_.pack(archive);
		Nose_.pack(archive);
		MouthCornerLeft_.pack(archive);
		MouthCornerRight_.pack(archive);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> is_tracked_;
		archive >> pitch_;
		archive >> yaw_;
		archive >> roll_;
		archive >> Happy_;
		archive >> Engaged_;
		archive >> WearingGlasses_;
		archive >> LeftEyeClosed_;
		archive >> RightEyeClosed_;
		archive >> MouthOpen_;
		archive >> MouthMoved_;
		archive >> LookingAway_;
		boundingBox_.unpack(archive);
		EyeLeft_.unpack(archive);
		EyeRight_.unpack(archive);
		Nose_.unpack(archive);
		MouthCornerLeft_.unpack(archive);
		MouthCornerRight_.unpack(archive);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectFaceMessage)
};

// ####################################################################################################
class KinectFacesMessage : public MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage>
{
public:
	typedef MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	KinectFacesMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectFacesMessage)
};

// ####################################################################################################

class KinectHDFaceMessage : public SerializableInterface
{
public:

	uint8_t is_tracked_;

	// ====================================================================================================
	KinectHDFaceMessage()
		:
		is_tracked_(0)
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << is_tracked_;
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> is_tracked_;
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectHDFaceMessage)
};

// ####################################################################################################
class KinectHDFacesMessage : public MultiMessage<VectorMessage<KinectHDFaceMessage>, TimeStampMessage>
{
public:
	typedef MultiMessage<VectorMessage<KinectHDFaceMessage>, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	KinectHDFacesMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectHDFacesMessage)
};

// ####################################################################################################
class KinectSpeechPhraseMessage : public SerializableInterface
{
public:
    std::string tag_;
    float confidence_;

    // ====================================================================================================
    KinectSpeechPhraseMessage()
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << tag_;
        archive << static_cast<float>( confidence_ );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> tag_;
        archive >> confidence_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectSpeechPhraseMessage )
};

// ####################################################################################################
class KinectSpeechMessage : public MultiMessage<VectorMessage<KinectSpeechPhraseMessage>, TimeStampMessage>
{
public:
    typedef MultiMessage<VectorMessage<KinectSpeechPhraseMessage>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectSpeechMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectSpeechMessage )
};

#endif // _MESSAGES_KINECTMESSAGES_H_
