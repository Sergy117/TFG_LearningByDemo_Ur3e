;; Auto-generated. Do not edit!


(when (boundp 'onrobot_gripper_msgs::OnrobotGripperCommandAction)
  (if (not (find-package "ONROBOT_GRIPPER_MSGS"))
    (make-package "ONROBOT_GRIPPER_MSGS"))
  (shadow 'OnrobotGripperCommandAction (find-package "ONROBOT_GRIPPER_MSGS")))
(unless (find-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMANDACTION")
  (make-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMANDACTION"))

(in-package "ROS")
;;//! \htmlinclude OnrobotGripperCommandAction.msg.html


(defclass onrobot_gripper_msgs::OnrobotGripperCommandAction
  :super ros::object
  :slots (_action_goal _action_result _action_feedback ))

(defmethod onrobot_gripper_msgs::OnrobotGripperCommandAction
  (:init
   (&key
    ((:action_goal __action_goal) (instance onrobot_gripper_msgs::OnrobotGripperCommandActionGoal :init))
    ((:action_result __action_result) (instance onrobot_gripper_msgs::OnrobotGripperCommandActionResult :init))
    ((:action_feedback __action_feedback) (instance onrobot_gripper_msgs::OnrobotGripperCommandActionFeedback :init))
    )
   (send-super :init)
   (setq _action_goal __action_goal)
   (setq _action_result __action_result)
   (setq _action_feedback __action_feedback)
   self)
  (:action_goal
   (&rest __action_goal)
   (if (keywordp (car __action_goal))
       (send* _action_goal __action_goal)
     (progn
       (if __action_goal (setq _action_goal (car __action_goal)))
       _action_goal)))
  (:action_result
   (&rest __action_result)
   (if (keywordp (car __action_result))
       (send* _action_result __action_result)
     (progn
       (if __action_result (setq _action_result (car __action_result)))
       _action_result)))
  (:action_feedback
   (&rest __action_feedback)
   (if (keywordp (car __action_feedback))
       (send* _action_feedback __action_feedback)
     (progn
       (if __action_feedback (setq _action_feedback (car __action_feedback)))
       _action_feedback)))
  (:serialization-length
   ()
   (+
    ;; onrobot_gripper_msgs/OnrobotGripperCommandActionGoal _action_goal
    (send _action_goal :serialization-length)
    ;; onrobot_gripper_msgs/OnrobotGripperCommandActionResult _action_result
    (send _action_result :serialization-length)
    ;; onrobot_gripper_msgs/OnrobotGripperCommandActionFeedback _action_feedback
    (send _action_feedback :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; onrobot_gripper_msgs/OnrobotGripperCommandActionGoal _action_goal
       (send _action_goal :serialize s)
     ;; onrobot_gripper_msgs/OnrobotGripperCommandActionResult _action_result
       (send _action_result :serialize s)
     ;; onrobot_gripper_msgs/OnrobotGripperCommandActionFeedback _action_feedback
       (send _action_feedback :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; onrobot_gripper_msgs/OnrobotGripperCommandActionGoal _action_goal
     (send _action_goal :deserialize buf ptr-) (incf ptr- (send _action_goal :serialization-length))
   ;; onrobot_gripper_msgs/OnrobotGripperCommandActionResult _action_result
     (send _action_result :deserialize buf ptr-) (incf ptr- (send _action_result :serialization-length))
   ;; onrobot_gripper_msgs/OnrobotGripperCommandActionFeedback _action_feedback
     (send _action_feedback :deserialize buf ptr-) (incf ptr- (send _action_feedback :serialization-length))
   ;;
   self)
  )

(setf (get onrobot_gripper_msgs::OnrobotGripperCommandAction :md5sum-) "950b2a6ebe831f5d4f4ceaba3d8be01e")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommandAction :datatype-) "onrobot_gripper_msgs/OnrobotGripperCommandAction")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommandAction :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

OnrobotGripperCommandActionGoal action_goal
OnrobotGripperCommandActionResult action_result
OnrobotGripperCommandActionFeedback action_feedback

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandActionGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalID goal_id
OnrobotGripperCommandGoal goal

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

================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id


================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
onrobot_gripper_msgs/OnrobotGripperCommand command

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommand
float64 position
float64 max_effort

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandActionResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
OnrobotGripperCommandResult result

================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text


================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandActionFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
OnrobotGripperCommandFeedback feedback

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommandFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint


")



(provide :onrobot_gripper_msgs/OnrobotGripperCommandAction "950b2a6ebe831f5d4f4ceaba3d8be01e")


