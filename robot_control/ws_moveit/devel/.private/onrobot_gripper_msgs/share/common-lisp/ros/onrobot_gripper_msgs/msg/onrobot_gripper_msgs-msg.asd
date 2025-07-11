
(cl:in-package :asdf)

(defsystem "onrobot_gripper_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointControllerState" :depends-on ("_package_JointControllerState"))
    (:file "_package_JointControllerState" :depends-on ("_package"))
    (:file "OnrobotGripperCommand" :depends-on ("_package_OnrobotGripperCommand"))
    (:file "_package_OnrobotGripperCommand" :depends-on ("_package"))
    (:file "OnrobotGripperCommandAction" :depends-on ("_package_OnrobotGripperCommandAction"))
    (:file "_package_OnrobotGripperCommandAction" :depends-on ("_package"))
    (:file "OnrobotGripperCommandActionFeedback" :depends-on ("_package_OnrobotGripperCommandActionFeedback"))
    (:file "_package_OnrobotGripperCommandActionFeedback" :depends-on ("_package"))
    (:file "OnrobotGripperCommandActionGoal" :depends-on ("_package_OnrobotGripperCommandActionGoal"))
    (:file "_package_OnrobotGripperCommandActionGoal" :depends-on ("_package"))
    (:file "OnrobotGripperCommandActionResult" :depends-on ("_package_OnrobotGripperCommandActionResult"))
    (:file "_package_OnrobotGripperCommandActionResult" :depends-on ("_package"))
    (:file "OnrobotGripperCommandFeedback" :depends-on ("_package_OnrobotGripperCommandFeedback"))
    (:file "_package_OnrobotGripperCommandFeedback" :depends-on ("_package"))
    (:file "OnrobotGripperCommandGoal" :depends-on ("_package_OnrobotGripperCommandGoal"))
    (:file "_package_OnrobotGripperCommandGoal" :depends-on ("_package"))
    (:file "OnrobotGripperCommandResult" :depends-on ("_package_OnrobotGripperCommandResult"))
    (:file "_package_OnrobotGripperCommandResult" :depends-on ("_package"))
  ))