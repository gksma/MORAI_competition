// Auto-generated. Do not edit!

// (in-package prediction.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackedObjectPose = require('./TrackedObjectPose.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackedObjectPoseList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose_list = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose_list')) {
        this.pose_list = initObj.pose_list
      }
      else {
        this.pose_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedObjectPoseList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose_list]
    // Serialize the length for message field [pose_list]
    bufferOffset = _serializer.uint32(obj.pose_list.length, buffer, bufferOffset);
    obj.pose_list.forEach((val) => {
      bufferOffset = TrackedObjectPose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedObjectPoseList
    let len;
    let data = new TrackedObjectPoseList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_list]
    // Deserialize array length for message field [pose_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pose_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose_list[i] = TrackedObjectPose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 52 * object.pose_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'prediction/TrackedObjectPoseList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3513663427657a6b21a11cd5a7988b35';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    TrackedObjectPose[] pose_list
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: prediction/TrackedObjectPose
    int32 unique_id
    
    TrackedPoint pose
    
    ================================================================================
    MSG: prediction/TrackedPoint
    float64 x
    float64 y
    float64 v
    float64 a
    float64 theta
    float64 theta_rate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackedObjectPoseList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose_list !== undefined) {
      resolved.pose_list = new Array(msg.pose_list.length);
      for (let i = 0; i < resolved.pose_list.length; ++i) {
        resolved.pose_list[i] = TrackedObjectPose.Resolve(msg.pose_list[i]);
      }
    }
    else {
      resolved.pose_list = []
    }

    return resolved;
    }
};

module.exports = TrackedObjectPoseList;
