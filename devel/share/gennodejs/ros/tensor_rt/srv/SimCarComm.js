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

class SimCarCommRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.room = null;
      this.class = null;
    }
    else {
      if (initObj.hasOwnProperty('room')) {
        this.room = initObj.room
      }
      else {
        this.room = 0;
      }
      if (initObj.hasOwnProperty('class')) {
        this.class = initObj.class
      }
      else {
        this.class = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimCarCommRequest
    // Serialize message field [room]
    bufferOffset = _serializer.int32(obj.room, buffer, bufferOffset);
    // Serialize message field [class]
    bufferOffset = _serializer.int32(obj.class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimCarCommRequest
    let len;
    let data = new SimCarCommRequest(null);
    // Deserialize message field [room]
    data.room = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [class]
    data.class = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tensor_rt/SimCarCommRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e328dd30e496c2742191cf878109dfb1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 room   #目标所在房间  
    int32 class  #目标类别
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimCarCommRequest(null);
    if (msg.room !== undefined) {
      resolved.room = msg.room;
    }
    else {
      resolved.room = 0
    }

    if (msg.class !== undefined) {
      resolved.class = msg.class;
    }
    else {
      resolved.class = 0
    }

    return resolved;
    }
};

class SimCarCommResponse {
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
    // Serializes a message object of type SimCarCommResponse
    // Serialize message field [target_class]
    bufferOffset = _serializer.int32(obj.target_class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimCarCommResponse
    let len;
    let data = new SimCarCommResponse(null);
    // Deserialize message field [target_class]
    data.target_class = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tensor_rt/SimCarCommResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e88f1a029f311d014085879de9fb23af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 target_class  #实际环境中的目标类别
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimCarCommResponse(null);
    if (msg.target_class !== undefined) {
      resolved.target_class = msg.target_class;
    }
    else {
      resolved.target_class = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SimCarCommRequest,
  Response: SimCarCommResponse,
  md5sum() { return '932cc26da5c945312b522ef7ce57d5c8'; },
  datatype() { return 'tensor_rt/SimCarComm'; }
};
