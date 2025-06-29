// Auto-generated. Do not edit!

// (in-package tensor_rt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msg_2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.room = null;
      this.detected_class = null;
    }
    else {
      if (initObj.hasOwnProperty('room')) {
        this.room = initObj.room
      }
      else {
        this.room = 0;
      }
      if (initObj.hasOwnProperty('detected_class')) {
        this.detected_class = initObj.detected_class
      }
      else {
        this.detected_class = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msg_2
    // Serialize message field [room]
    bufferOffset = _serializer.int32(obj.room, buffer, bufferOffset);
    // Serialize message field [detected_class]
    bufferOffset = _serializer.int32(obj.detected_class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msg_2
    let len;
    let data = new msg_2(null);
    // Deserialize message field [room]
    data.room = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [detected_class]
    data.detected_class = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tensor_rt/msg_2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd4962e33f389591ae16435b9eda70efd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 room
    int32 detected_class
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msg_2(null);
    if (msg.room !== undefined) {
      resolved.room = msg.room;
    }
    else {
      resolved.room = 0
    }

    if (msg.detected_class !== undefined) {
      resolved.detected_class = msg.detected_class;
    }
    else {
      resolved.detected_class = 0
    }

    return resolved;
    }
};

module.exports = msg_2;
