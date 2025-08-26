
(cl:in-package :asdf)

(defsystem "prediction-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PredictedObjectPath" :depends-on ("_package_PredictedObjectPath"))
    (:file "_package_PredictedObjectPath" :depends-on ("_package"))
    (:file "PredictedObjectPathList" :depends-on ("_package_PredictedObjectPathList"))
    (:file "_package_PredictedObjectPathList" :depends-on ("_package"))
    (:file "TrackedObjectPose" :depends-on ("_package_TrackedObjectPose"))
    (:file "_package_TrackedObjectPose" :depends-on ("_package"))
    (:file "TrackedObjectPoseList" :depends-on ("_package_TrackedObjectPoseList"))
    (:file "_package_TrackedObjectPoseList" :depends-on ("_package"))
    (:file "TrackedPoint" :depends-on ("_package_TrackedPoint"))
    (:file "_package_TrackedPoint" :depends-on ("_package"))
  ))