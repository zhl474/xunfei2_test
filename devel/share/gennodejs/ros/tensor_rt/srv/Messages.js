// Auto-generated. Do not edit!

// (in-package tensor_rt.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MessagesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object = null;
    }
    else {
      if (initObj.hasOwnProperty('object')) {
        this.object = initObj.object
      }
      else {
        this.object = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MessagesRequest
    // Serialize message field [object]
    bufferOffset = _serializer.int32(obj.object, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MessagesRequest
    let len;
    let data = new MessagesRequest(null);
    // Deserialize message field [object]
    data.object = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tensor_rt/MessagesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '697f5f1b63c5a2c12b7d7f348a45ff41';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 object
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MessagesRequest(null);
    if (msg.object !== undefined) {
      resolved.object = msg.object;
    }
    else {
      resolved.object = 0
    }

    return resolved;
    }
};

class MessagesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MessagesResponse
    // Serialize message field [result]
    bufferOffset = _arraySerializer.int32(obj.result, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MessagesResponse
    let len;
    let data = new MessagesResponse(null);
    // Deserialize message field [result]
    data.result = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tensor_rt/MessagesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '735d5bb657ca4356bf24ff5eb86bf466';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MessagesResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = []
    }

    return resolved;
    }
};

module.exports = {
  Request: MessagesRequest,
  Response: MessagesResponse,
  md5sum() { return 'bdb4ecf8e91e5c89bb2f7e7af5cf8ea3'; },
  datatype() { return 'tensor_rt/Messages'; }
};
