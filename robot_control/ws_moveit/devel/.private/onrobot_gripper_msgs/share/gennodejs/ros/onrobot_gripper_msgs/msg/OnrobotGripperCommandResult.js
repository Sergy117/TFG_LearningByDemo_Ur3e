// Auto-generated. Do not edit!

// (in-package onrobot_gripper_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OnrobotGripperCommandResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.effort = null;
      this.stalled = null;
      this.reached_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = 0.0;
      }
      if (initObj.hasOwnProperty('stalled')) {
        this.stalled = initObj.stalled
      }
      else {
        this.stalled = false;
      }
      if (initObj.hasOwnProperty('reached_goal')) {
        this.reached_goal = initObj.reached_goal
      }
      else {
        this.reached_goal = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OnrobotGripperCommandResult
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [effort]
    bufferOffset = _serializer.float64(obj.effort, buffer, bufferOffset);
    // Serialize message field [stalled]
    bufferOffset = _serializer.bool(obj.stalled, buffer, bufferOffset);
    // Serialize message field [reached_goal]
    bufferOffset = _serializer.bool(obj.reached_goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OnrobotGripperCommandResult
    let len;
    let data = new OnrobotGripperCommandResult(null);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [effort]
    data.effort = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stalled]
    data.stalled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reached_goal]
    data.reached_goal = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'onrobot_gripper_msgs/OnrobotGripperCommandResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4cbff56d3562bcf113da5a5adeef91f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    float64 position  # The current gripper gap size (in meters)
    float64 effort    # The current effort exerted (in Newtons)
    bool stalled      # True iff the gripper is exerting max effort and not moving
    bool reached_goal # True iff the gripper position has reached the commanded setpoint
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OnrobotGripperCommandResult(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = 0.0
    }

    if (msg.stalled !== undefined) {
      resolved.stalled = msg.stalled;
    }
    else {
      resolved.stalled = false
    }

    if (msg.reached_goal !== undefined) {
      resolved.reached_goal = msg.reached_goal;
    }
    else {
      resolved.reached_goal = false
    }

    return resolved;
    }
};

module.exports = OnrobotGripperCommandResult;
