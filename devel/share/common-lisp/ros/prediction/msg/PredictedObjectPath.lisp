; Auto-generated. Do not edit!


(cl:in-package prediction-msg)


;//! \htmlinclude PredictedObjectPath.msg.html

(cl:defclass <PredictedObjectPath> (roslisp-msg-protocol:ros-message)
  ((unique_id
    :reader unique_id
    :initarg :unique_id
    :type cl:integer
    :initform 0)
   (path
    :reader path
    :initarg :path
    :type (cl:vector prediction-msg:TrackedPoint)
   :initform (cl:make-array 0 :element-type 'prediction-msg:TrackedPoint :initial-element (cl:make-instance 'prediction-msg:TrackedPoint))))
)

(cl:defclass PredictedObjectPath (<PredictedObjectPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictedObjectPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictedObjectPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name prediction-msg:<PredictedObjectPath> is deprecated: use prediction-msg:PredictedObjectPath instead.")))

(cl:ensure-generic-function 'unique_id-val :lambda-list '(m))
(cl:defmethod unique_id-val ((m <PredictedObjectPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:unique_id-val is deprecated.  Use prediction-msg:unique_id instead.")
  (unique_id m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <PredictedObjectPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:path-val is deprecated.  Use prediction-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictedObjectPath>) ostream)
  "Serializes a message object of type '<PredictedObjectPath>"
  (cl:let* ((signed (cl:slot-value msg 'unique_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictedObjectPath>) istream)
  "Deserializes a message object of type '<PredictedObjectPath>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'unique_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'prediction-msg:TrackedPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictedObjectPath>)))
  "Returns string type for a message object of type '<PredictedObjectPath>"
  "prediction/PredictedObjectPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictedObjectPath)))
  "Returns string type for a message object of type 'PredictedObjectPath"
  "prediction/PredictedObjectPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictedObjectPath>)))
  "Returns md5sum for a message object of type '<PredictedObjectPath>"
  "a136606ffb64a5c19dfc192304ca3b46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictedObjectPath)))
  "Returns md5sum for a message object of type 'PredictedObjectPath"
  "a136606ffb64a5c19dfc192304ca3b46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictedObjectPath>)))
  "Returns full string definition for message of type '<PredictedObjectPath>"
  (cl:format cl:nil "int32 unique_id~%~%TrackedPoint[] path~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictedObjectPath)))
  "Returns full string definition for message of type 'PredictedObjectPath"
  (cl:format cl:nil "int32 unique_id~%~%TrackedPoint[] path~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictedObjectPath>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictedObjectPath>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictedObjectPath
    (cl:cons ':unique_id (unique_id msg))
    (cl:cons ':path (path msg))
))
