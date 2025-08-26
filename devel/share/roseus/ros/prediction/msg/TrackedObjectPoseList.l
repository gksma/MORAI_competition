;; Auto-generated. Do not edit!


(when (boundp 'prediction::TrackedObjectPoseList)
  (if (not (find-package "PREDICTION"))
    (make-package "PREDICTION"))
  (shadow 'TrackedObjectPoseList (find-package "PREDICTION")))
(unless (find-package "PREDICTION::TRACKEDOBJECTPOSELIST")
  (make-package "PREDICTION::TRACKEDOBJECTPOSELIST"))

(in-package "ROS")
;;//! \htmlinclude TrackedObjectPoseList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass prediction::TrackedObjectPoseList
  :super ros::object
  :slots (_header _pose_list ))

(defmethod prediction::TrackedObjectPoseList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose_list __pose_list) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose_list __pose_list)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose_list
   (&rest __pose_list)
   (if (keywordp (car __pose_list))
       (send* _pose_list __pose_list)
     (progn
       (if __pose_list (setq _pose_list (car __pose_list)))
       _pose_list)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; prediction/TrackedObjectPose[] _pose_list
    (apply #'+ (send-all _pose_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; prediction/TrackedObjectPose[] _pose_list
     (write-long (length _pose_list) s)
     (dolist (elem _pose_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; prediction/TrackedObjectPose[] _pose_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_list (let (r) (dotimes (i n) (push (instance prediction::TrackedObjectPose :init) r)) r))
     (dolist (elem- _pose_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get prediction::TrackedObjectPoseList :md5sum-) "3513663427657a6b21a11cd5a7988b35")
(setf (get prediction::TrackedObjectPoseList :datatype-) "prediction/TrackedObjectPoseList")
(setf (get prediction::TrackedObjectPoseList :definition-)
      "Header header

TrackedObjectPose[] pose_list

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
MSG: prediction/TrackedObjectPose
int32 unique_id

TrackedPoint pose

================================================================================
MSG: prediction/TrackedPoint
float64 x
float64 y
float64 v
float64 a
float64 theta
float64 theta_rate

")



(provide :prediction/TrackedObjectPoseList "3513663427657a6b21a11cd5a7988b35")


