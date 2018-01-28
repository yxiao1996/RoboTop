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

class chassis_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.chassis = null;
      this.str = null;
    }
    else {
      if (initObj.hasOwnProperty('chassis')) {
        this.chassis = initObj.chassis
      }
      else {
        this.chassis = [];
      }
      if (initObj.hasOwnProperty('str')) {
        this.str = initObj.str
      }
      else {
        this.str = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type chassis_msg
    // Serialize message field [chassis]
    bufferOffset = _arraySerializer.float64(obj.chassis, buffer, bufferOffset, null);
    // Serialize message field [str]
    bufferOffset = _serializer.string(obj.str, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type chassis_msg
    let len;
    let data = new chassis_msg(null);
    // Deserialize message field [chassis]
    data.chassis = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [str]
    data.str = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.chassis.length;
    length += object.str.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robocon_msgs/chassis_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a899e5eecd9f022d7c1366dd45f69c78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] chassis
    string str
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new chassis_msg(null);
    if (msg.chassis !== undefined) {
      resolved.chassis = msg.chassis;
    }
    else {
      resolved.chassis = []
    }

    if (msg.str !== undefined) {
      resolved.str = msg.str;
    }
    else {
      resolved.str = ''
    }

    return resolved;
    }
};

module.exports = chassis_msg;
