// Auto-generated. Do not edit!

// (in-package prediction.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PredictedObjectPath = require('./PredictedObjectPath.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PredictedObjectPathList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.path_list = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('path_list')) {
        this.path_list = initObj.path_list
      }
      else {
        this.path_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PredictedObjectPathList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [path_list]
    // Serialize the length for message field [path_list]
    bufferOffset = _serializer.uint32(obj.path_list.length, buffer, bufferOffset);
    obj.path_list.forEach((val) => {
      bufferOffset = PredictedObjectPath.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PredictedObjectPathList
    let len;
    let data = new PredictedObjectPathList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [path_list]
    // Deserialize array length for message field [path_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path_list[i] = PredictedObjectPath.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.path_list.forEach((val) => {
      length += PredictedObjectPath.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'prediction/PredictedObjectPathList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e648b9acebe43c8c9a3af0882ce9d66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    PredictedObjectPath[] path_list
    
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
    MSG: prediction/PredictedObjectPath
    int32 unique_id
    
    TrackedPoint[] path
    
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
    const resolved = new PredictedObjectPathList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.path_list !== undefined) {
      resolved.path_list = new Array(msg.path_list.length);
      for (let i = 0; i < resolved.path_list.length; ++i) {
        resolved.path_list[i] = PredictedObjectPath.Resolve(msg.path_list[i]);
      }
    }
    else {
      resolved.path_list = []
    }

    return resolved;
    }
};

module.exports = PredictedObjectPathList;
