// Auto-generated. Do not edit!

// (in-package robocon_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class remoter_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.remote_data = null;
      this.attitude = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('remote_data')) {
        this.remote_data = initObj.remote_data
      }
      else {
        this.remote_data = [];
      }
      if (initObj.hasOwnProperty('attitude')) {
        this.attitude = initObj.attitude
      }
      else {
        this.attitude = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type remoter_msg
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float32(obj.position, buffer, bufferOffset, null);
    // Serialize message field [remote_data]
    bufferOffset = _arraySerializer.float32(obj.remote_data, buffer, bufferOffset, null);
    // Serialize message field [attitude]
    bufferOffset = _arraySerializer.float64(obj.attitude, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type remoter_msg
    let len;
    let data = new remoter_msg(null);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remote_data]
    data.remote_data = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [attitude]
    data.attitude = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.position.length;
    length += 4 * object.remote_data.length;
    length += 8 * object.attitude.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robocon_msgs/remoter_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75585e5d82b120054891679ec4f80561';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] position
    float32[] remote_data
    float64[] attitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new remoter_msg(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.remote_data !== undefined) {
      resolved.remote_data = msg.remote_data;
    }
    else {
      resolved.remote_data = []
    }

    if (msg.attitude !== undefined) {
      resolved.attitude = msg.attitude;
    }
    else {
      resolved.attitude = []
    }

    return resolved;
    }
};

module.exports = remoter_msg;
