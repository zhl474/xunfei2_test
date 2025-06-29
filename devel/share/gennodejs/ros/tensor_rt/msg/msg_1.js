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

class msg_1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_class = null;
    }
    else {
      if (initObj.hasOwnProperty('target_class')) {
        this.target_class = initObj.target_class
      }
      else {
        this.target_class = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msg_1
    // Serialize message field [target_class]
    bufferOffset = _serializer.uint16(obj.target_class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msg_1
    let len;
    let data = new msg_1(null);
    // Deserialize message field [target_class]
    data.target_class = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tensor_rt/msg_1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '997878a101ab058a18257fccc8baf249';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 target_class
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msg_1(null);
    if (msg.target_class !== undefined) {
      resolved.target_class = msg.target_class;
    }
    else {
      resolved.target_class = 0
    }

    return resolved;
    }
};

module.exports = msg_1;
