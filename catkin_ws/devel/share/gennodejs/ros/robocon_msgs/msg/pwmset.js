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

class pwmset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pwm = null;
    }
    else {
      if (initObj.hasOwnProperty('pwm')) {
        this.pwm = initObj.pwm
      }
      else {
        this.pwm = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pwmset
    // Serialize message field [pwm]
    bufferOffset = _arraySerializer.float32(obj.pwm, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pwmset
    let len;
    let data = new pwmset(null);
    // Deserialize message field [pwm]
    data.pwm = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pwm.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robocon_msgs/pwmset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8d84643029731b92d628f398337d291';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] pwm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pwmset(null);
    if (msg.pwm !== undefined) {
      resolved.pwm = msg.pwm;
    }
    else {
      resolved.pwm = []
    }

    return resolved;
    }
};

module.exports = pwmset;
