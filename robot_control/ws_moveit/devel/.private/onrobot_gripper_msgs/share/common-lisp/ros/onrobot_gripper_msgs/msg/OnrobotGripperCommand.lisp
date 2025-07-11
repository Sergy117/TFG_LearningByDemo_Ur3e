; Auto-generated. Do not edit!


(cl:in-package onrobot_gripper_msgs-msg)


;//! \htmlinclude OnrobotGripperCommand.msg.html

(cl:defclass <OnrobotGripperCommand> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (max_effort
    :reader max_effort
    :initarg :max_effort
    :type cl:float
    :initform 0.0))
)

(cl:defclass OnrobotGripperCommand (<OnrobotGripperCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnrobotGripperCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnrobotGripperCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name onrobot_gripper_msgs-msg:<OnrobotGripperCommand> is deprecated: use onrobot_gripper_msgs-msg:OnrobotGripperCommand instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <OnrobotGripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onrobot_gripper_msgs-msg:position-val is deprecated.  Use onrobot_gripper_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'max_effort-val :lambda-list '(m))
(cl:defmethod max_effort-val ((m <OnrobotGripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onrobot_gripper_msgs-msg:max_effort-val is deprecated.  Use onrobot_gripper_msgs-msg:max_effort instead.")
  (max_effort m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnrobotGripperCommand>) ostream)
  "Serializes a message object of type '<OnrobotGripperCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnrobotGripperCommand>) istream)
  "Deserializes a message object of type '<OnrobotGripperCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_effort) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnrobotGripperCommand>)))
  "Returns string type for a message object of type '<OnrobotGripperCommand>"
  "onrobot_gripper_msgs/OnrobotGripperCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnrobotGripperCommand)))
  "Returns string type for a message object of type 'OnrobotGripperCommand"
  "onrobot_gripper_msgs/OnrobotGripperCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnrobotGripperCommand>)))
  "Returns md5sum for a message object of type '<OnrobotGripperCommand>"
  "680acaff79486f017132a7f198d40f08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnrobotGripperCommand)))
  "Returns md5sum for a message object of type 'OnrobotGripperCommand"
  "680acaff79486f017132a7f198d40f08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnrobotGripperCommand>)))
  "Returns full string definition for message of type '<OnrobotGripperCommand>"
  (cl:format cl:nil "float64 position~%float64 max_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnrobotGripperCommand)))
  "Returns full string definition for message of type 'OnrobotGripperCommand"
  (cl:format cl:nil "float64 position~%float64 max_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnrobotGripperCommand>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnrobotGripperCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'OnrobotGripperCommand
    (cl:cons ':position (position msg))
    (cl:cons ':max_effort (max_effort msg))
))
