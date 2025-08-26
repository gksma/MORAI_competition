;; Auto-generated. Do not edit!


(when (boundp 'prediction::PredictedObjectPathList)
  (if (not (find-package "PREDICTION"))
    (make-package "PREDICTION"))
  (shadow 'PredictedObjectPathList (find-package "PREDICTION")))
(unless (find-package "PREDICTION::PREDICTEDOBJECTPATHLIST")
  (make-package "PREDICTION::PREDICTEDOBJECTPATHLIST"))

(in-package "ROS")
;;//! \htmlinclude PredictedObjectPathList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass prediction::PredictedObjectPathList
  :super ros::object
  :slots (_header _path_list ))

(defmethod prediction::PredictedObjectPathList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:path_list __path_list) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _path_list __path_list)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:path_list
   (&rest __path_list)
   (if (keywordp (car __path_list))
       (send* _path_list __path_list)
     (progn
       (if __path_list (setq _path_list (car __path_list)))
       _path_list)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; prediction/PredictedObjectPath[] _path_list
    (apply #'+ (send-all _path_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; prediction/PredictedObjectPath[] _path_list
     (write-long (length _path_list) s)
     (dolist (elem _path_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; prediction/PredictedObjectPath[] _path_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_list (let (r) (dotimes (i n) (push (instance prediction::PredictedObjectPath :init) r)) r))
     (dolist (elem- _path_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get prediction::PredictedObjectPathList :md5sum-) "2e648b9acebe43c8c9a3af0882ce9d66")
(setf (get prediction::PredictedObjectPathList :datatype-) "prediction/PredictedObjectPathList")
(setf (get prediction::PredictedObjectPathList :definition-)
      "Header header

PredictedObjectPath[] path_list

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
MSG: prediction/PredictedObjectPath
int32 unique_id

TrackedPoint[] path

================================================================================
MSG: prediction/TrackedPoint
float64 x
float64 y
float64 v
float64 a
float64 theta
float64 theta_rate

")



(provide :prediction/PredictedObjectPathList "2e648b9acebe43c8c9a3af0882ce9d66")


