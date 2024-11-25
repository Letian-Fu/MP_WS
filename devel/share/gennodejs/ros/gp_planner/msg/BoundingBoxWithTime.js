// Auto-generated. Do not edit!

// (in-package gp_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BoundingBoxWithTime {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_x = null;
      this.right_x = null;
      this.up_y = null;
      this.down_y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_x')) {
        this.left_x = initObj.left_x
      }
      else {
        this.left_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_x')) {
        this.right_x = initObj.right_x
      }
      else {
        this.right_x = 0.0;
      }
      if (initObj.hasOwnProperty('up_y')) {
        this.up_y = initObj.up_y
      }
      else {
        this.up_y = 0.0;
      }
      if (initObj.hasOwnProperty('down_y')) {
        this.down_y = initObj.down_y
      }
      else {
        this.down_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBoxWithTime
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left_x]
    bufferOffset = _serializer.float32(obj.left_x, buffer, bufferOffset);
    // Serialize message field [right_x]
    bufferOffset = _serializer.float32(obj.right_x, buffer, bufferOffset);
    // Serialize message field [up_y]
    bufferOffset = _serializer.float32(obj.up_y, buffer, bufferOffset);
    // Serialize message field [down_y]
    bufferOffset = _serializer.float32(obj.down_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBoxWithTime
    let len;
    let data = new BoundingBoxWithTime(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_x]
    data.left_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_x]
    data.right_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [up_y]
    data.up_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [down_y]
    data.down_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gp_planner/BoundingBoxWithTime';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3f7bcaf4671d6882f7146f67396528cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 文件名：BoundingBoxWithTime.msg
    std_msgs/Header header
    float32 left_x   # 左边界的x坐标
    float32 right_x  # 右边界的x坐标
    float32 up_y     # 上边界的y坐标
    float32 down_y   # 下边界的y坐标
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBoxWithTime(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_x !== undefined) {
      resolved.left_x = msg.left_x;
    }
    else {
      resolved.left_x = 0.0
    }

    if (msg.right_x !== undefined) {
      resolved.right_x = msg.right_x;
    }
    else {
      resolved.right_x = 0.0
    }

    if (msg.up_y !== undefined) {
      resolved.up_y = msg.up_y;
    }
    else {
      resolved.up_y = 0.0
    }

    if (msg.down_y !== undefined) {
      resolved.down_y = msg.down_y;
    }
    else {
      resolved.down_y = 0.0
    }

    return resolved;
    }
};

module.exports = BoundingBoxWithTime;
