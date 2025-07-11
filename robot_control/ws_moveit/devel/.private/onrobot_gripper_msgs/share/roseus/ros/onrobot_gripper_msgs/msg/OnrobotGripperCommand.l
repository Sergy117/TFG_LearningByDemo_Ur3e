;; Auto-generated. Do not edit!


(when (boundp 'onrobot_gripper_msgs::OnrobotGripperCommand)
  (if (not (find-package "ONROBOT_GRIPPER_MSGS"))
    (make-package "ONROBOT_GRIPPER_MSGS"))
  (shadow 'OnrobotGripperCommand (find-package "ONROBOT_GRIPPER_MSGS")))
(unless (find-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMAND")
  (make-package "ONROBOT_GRIPPER_MSGS::ONROBOTGRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude OnrobotGripperCommand.msg.html


(defclass onrobot_gripper_msgs::OnrobotGripperCommand
  :super ros::object
  :slots (_position _max_effort ))

(defmethod onrobot_gripper_msgs::OnrobotGripperCommand
  (:init
   (&key
    ((:position __position) 0.0)
    ((:max_effort __max_effort) 0.0)
    )
   (send-super :init)
   (setq _position (float __position))
   (setq _max_effort (float __max_effort))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:max_effort
   (&optional __max_effort)
   (if __max_effort (setq _max_effort __max_effort)) _max_effort)
  (:serialization-length
   ()
   (+
    ;; float64 _position
    8
    ;; float64 _max_effort
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_effort
       (sys::poke _max_effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_effort
     (setq _max_effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get onrobot_gripper_msgs::OnrobotGripperCommand :md5sum-) "680acaff79486f017132a7f198d40f08")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommand :datatype-) "onrobot_gripper_msgs/OnrobotGripperCommand")
(setf (get onrobot_gripper_msgs::OnrobotGripperCommand :definition-)
      "float64 position
float64 max_effort

")



(provide :onrobot_gripper_msgs/OnrobotGripperCommand "680acaff79486f017132a7f198d40f08")


