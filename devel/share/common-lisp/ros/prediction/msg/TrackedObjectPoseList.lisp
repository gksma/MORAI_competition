; Auto-generated. Do not edit!


(cl:in-package prediction-msg)


;//! \htmlinclude TrackedObjectPoseList.msg.html

(cl:defclass <TrackedObjectPoseList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose_list
    :reader pose_list
    :initarg :pose_list
    :type (cl:vector prediction-msg:TrackedObjectPose)
   :initform (cl:make-array 0 :element-type 'prediction-msg:TrackedObjectPose :initial-element (cl:make-instance 'prediction-msg:TrackedObjectPose))))
)

(cl:defclass TrackedObjectPoseList (<TrackedObjectPoseList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedObjectPoseList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedObjectPoseList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name prediction-msg:<TrackedObjectPoseList> is deprecated: use prediction-msg:TrackedObjectPoseList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedObjectPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:header-val is deprecated.  Use prediction-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose_list-val :lambda-list '(m))
(cl:defmethod pose_list-val ((m <TrackedObjectPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:pose_list-val is deprecated.  Use prediction-msg:pose_list instead.")
  (pose_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedObjectPoseList>) ostream)
  "Serializes a message object of type '<TrackedObjectPoseList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pose_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pose_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedObjectPoseList>) istream)
  "Deserializes a message object of type '<TrackedObjectPoseList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pose_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pose_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'prediction-msg:TrackedObjectPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedObjectPoseList>)))
  "Returns string type for a message object of type '<TrackedObjectPoseList>"
  "prediction/TrackedObjectPoseList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedObjectPoseList)))
  "Returns string type for a message object of type 'TrackedObjectPoseList"
  "prediction/TrackedObjectPoseList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedObjectPoseList>)))
  "Returns md5sum for a message object of type '<TrackedObjectPoseList>"
  "3513663427657a6b21a11cd5a7988b35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedObjectPoseList)))
  "Returns md5sum for a message object of type 'TrackedObjectPoseList"
  "3513663427657a6b21a11cd5a7988b35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedObjectPoseList>)))
  "Returns full string definition for message of type '<TrackedObjectPoseList>"
  (cl:format cl:nil "Header header~%~%TrackedObjectPose[] pose_list~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: prediction/TrackedObjectPose~%int32 unique_id~%~%TrackedPoint pose~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedObjectPoseList)))
  "Returns full string definition for message of type 'TrackedObjectPoseList"
  (cl:format cl:nil "Header header~%~%TrackedObjectPose[] pose_list~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: prediction/TrackedObjectPose~%int32 unique_id~%~%TrackedPoint pose~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedObjectPoseList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pose_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedObjectPoseList>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedObjectPoseList
    (cl:cons ':header (header msg))
    (cl:cons ':pose_list (pose_list msg))
))
