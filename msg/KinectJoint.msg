uint8 JOINT_TYPE_SPINE_BASE = 0
uint8 JOINT_TYPE_SPINE_MID = 1
uint8 JOINT_TYPE_NECK = 2
uint8 JOINT_TYPE_HEAD = 3
uint8 JOINT_TYPE_SHOULDER_LEFT = 4
uint8 JOINT_TYPE_ELBOW_LEFT = 5
uint8 JOINT_TYPE_WRIST_LEFT = 6
uint8 JOINT_TYPE_HAND_LEFT = 7
uint8 JOINT_TYPE_SHOULDER_RIGHT = 8
uint8 JOINT_TYPE_ELBOW_RIGHT = 9
uint8 JOINT_TYPE_WRIST_RIGHT = 10
uint8 JOINT_TYPE_HAND_RIGHT = 11
uint8 JOINT_TYPE_HIP_LEFT = 12
uint8 JOINT_TYPE_KNEE_LEFT = 13
uint8 JOINT_TYPE_ANKLE_LEFT = 14
uint8 JOINT_TYPE_FOOT_LEFT = 15
uint8 JOINT_TYPE_HIP_RIGHT = 16
uint8 JOINT_TYPE_KNEE_RIGHT = 17
uint8 JOINT_TYPE_ANKLE_RIGHT = 18
uint8 JOINT_TYPE_FOOT_RIGHT = 19
uint8 JOINT_TYPE_SPINE_SHOULDER = 20
uint8 JOINT_TYPE_HANDTIP_LEFT = 21
uint8 JOINT_TYPE_THUMB_LEFT = 22
uint8 JOINT_TYPE_HANDTIP_RIGHT = 23
uint8 JOINT_TYPE_THUMB_RIGHT = 24

uint8 TRACKING_STATE_NOT_TRACKED = 0
uint8 TRACKING_STATE_INFERRED = 1
uint8 TRACKING_STATE_TRACKED = 2

geometry_msgs/Point tracker
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
uint8 joint_type
uint8 tracking_state

