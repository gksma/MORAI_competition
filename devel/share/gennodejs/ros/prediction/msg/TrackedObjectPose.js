// Auto-generated. Do not edit!

// (in-package prediction.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackedPoint = require('./TrackedPoint.js');

//-----------------------------------------------------------

class TrackedObjectPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.unique_id = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('unique_id')) {
        this.unique_id = initObj.unique_id
      }
      else {
        this.unique_id = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new TrackedPoint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedObjectPose
    // Serialize message field [unique_id]
    bufferOffset = _serializer.int32(obj.unique_id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = TrackedPoint.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedObjectPose
    let len;
    let data = new TrackedObjectPose(null);
    // Deserialize message field [unique_id]
    data.unique_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = TrackedPoint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'prediction/TrackedObjectPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7690af574bf077cf4e6a344bb466312f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new TrackedObjectPose(null);
    if (msg.unique_id !== undefined) {
      resolved.unique_id = msg.unique_id;
    }
    else {
      resolved.unique_id = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = TrackedPoint.Resolve(msg.pose)
    }
    else {
      resolved.pose = new TrackedPoint()
    }

    return resolved;
    }
};

module.exports = TrackedObjectPose;
