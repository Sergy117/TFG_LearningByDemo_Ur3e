; Auto-generated. Do not edit!


(cl:in-package gripper_action-msg)


;//! \htmlinclude gripperGoal.msg.html

(cl:defclass <gripperGoal> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (force
    :reader force
    :initarg :force
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass gripperGoal (<gripperGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripperGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripperGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gripper_action-msg:<gripperGoal> is deprecated: use gripper_action-msg:gripperGoal instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <gripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gripper_action-msg:width-val is deprecated.  Use gripper_action-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <gripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gripper_action-msg:speed-val is deprecated.  Use gripper_action-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <gripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gripper_action-msg:force-val is deprecated.  Use gripper_action-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <gripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gripper_action-msg:type-val is deprecated.  Use gripper_action-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripperGoal>) ostream)
  "Serializes a message object of type '<gripperGoal>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'force)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripperGoal>) istream)
  "Deserializes a message object of type '<gripperGoal>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'force) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripperGoal>)))
  "Returns string type for a message object of type '<gripperGoal>"
  "gripper_action/gripperGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripperGoal)))
  "Returns string type for a message object of type 'gripperGoal"
  "gripper_action/gripperGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripperGoal>)))
  "Returns md5sum for a message object of type '<gripperGoal>"
  "099fd3e83221576cc0767cfa847e1828")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripperGoal)))
  "Returns md5sum for a message object of type 'gripperGoal"
  "099fd3e83221576cc0767cfa847e1828")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripperGoal>)))
  "Returns full string definition for message of type '<gripperGoal>"
  (cl:format cl:nil "float32 width # [mm]~%int32 speed ~%int32 force ~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripperGoal)))
  "Returns full string definition for message of type 'gripperGoal"
  (cl:format cl:nil "float32 width # [mm]~%int32 speed ~%int32 force ~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripperGoal>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripperGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'gripperGoal
    (cl:cons ':width (width msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':force (force msg))
    (cl:cons ':type (type msg))
))
