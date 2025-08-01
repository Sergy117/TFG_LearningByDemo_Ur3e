;; Auto-generated. Do not edit!


(when (boundp 'onrobot_gripper_msgs::OnrobotGripperCommandGoal)
  (if (not (find-package "ONROBOT_GRIPPER_MSGS"))
    (make-package "ONROBOT_GRIPPER_MSGS"))
  (shadow 'OnrobotGripperCommandGoal (find-package "ONROBOT_GRIPPER_MSGS")))
(unless (find-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMANDGOAL")
  (make-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMANDGOAL"))

(in-package "ROS")
;;//! \htmlinclude OnrobotGripperCommandGoal.msg.html


(defclass onrobot_gripper_msgs::OnrobotGripperCommandGoal
  :super ros::object
  :slots (_command ))

(defmethod onrobot_gripper_msgs::OnrobotGripperCommandGoal
  (:init
   (&key
    ((:command __command) (instance onrobot_gripper_msgs::OnrobotGripperCommand :init))
    )
   (send-super :init)
   (setq _command __command)
   self)
  (:command
   (&rest __command)
   (if (keywordp (car __command))
       (send* _command __command)
     (progn
       (if __command (setq _command (car __command)))
       _command)))
  (:serialization-length
   ()
   (+
    ;; onrobot_gripper_msgs/OnrobotGripperCommand _command
    (send _command :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; onrobot_gripper_msgs/OnrobotGripperCommand _command
       (send _command :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; onrobot_gripper_msgs/OnrobotGripperCommand _command
     (send _command :deserialize buf ptr-) (incf ptr- (send _command :serialization-length))
   ;;
   self)
  )

(setf (get onrobot_gripper_msgs::OnrobotGripperCommandGoal :md5sum-) "86fd82f4ddc48a4cb6856cfa69217e43")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommandGoal :datatype-) "onrobot_gripper_msgs/OnrobotGripperCommandGoal")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommandGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
onrobot_gripper_msgs/OnrobotGripperCommand command

================================================================================
MSG: onrobot_gripper_msgs/OnrobotGripperCommand
float64 position
float64 max_effort

")



(provide :onrobot_gripper_msgs/OnrobotGripperCommandGoal "86fd82f4ddc48a4cb6856cfa69217e43")


