; Auto-generated. Do not edit!


(cl:in-package prediction-msg)


;//! \htmlinclude TrackedObjectPose.msg.html

(cl:defclass <TrackedObjectPose> (roslisp-msg-protocol:ros-message)
  ((unique_id
    :reader unique_id
    :initarg :unique_id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type prediction-msg:TrackedPoint
    :initform (cl:make-instance 'prediction-msg:TrackedPoint)))
)

(cl:defclass TrackedObjectPose (<TrackedObjectPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedObjectPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedObjectPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name prediction-msg:<TrackedObjectPose> is deprecated: use prediction-msg:TrackedObjectPose instead.")))

(cl:ensure-generic-function 'unique_id-val :lambda-list '(m))
(cl:defmethod unique_id-val ((m <TrackedObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:unique_id-val is deprecated.  Use prediction-msg:unique_id instead.")
  (unique_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TrackedObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prediction-msg:pose-val is deprecated.  Use prediction-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedObjectPose>) ostream)
  "Serializes a message object of type '<TrackedObjectPose>"
  (cl:let* ((signed (cl:slot-value msg 'unique_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedObjectPose>) istream)
  "Deserializes a message object of type '<TrackedObjectPose>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'unique_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedObjectPose>)))
  "Returns string type for a message object of type '<TrackedObjectPose>"
  "prediction/TrackedObjectPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedObjectPose)))
  "Returns string type for a message object of type 'TrackedObjectPose"
  "prediction/TrackedObjectPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedObjectPose>)))
  "Returns md5sum for a message object of type '<TrackedObjectPose>"
  "7690af574bf077cf4e6a344bb466312f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedObjectPose)))
  "Returns md5sum for a message object of type 'TrackedObjectPose"
  "7690af574bf077cf4e6a344bb466312f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedObjectPose>)))
  "Returns full string definition for message of type '<TrackedObjectPose>"
  (cl:format cl:nil "int32 unique_id~%~%TrackedPoint pose~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedObjectPose)))
  "Returns full string definition for message of type 'TrackedObjectPose"
  (cl:format cl:nil "int32 unique_id~%~%TrackedPoint pose~%~%================================================================================~%MSG: prediction/TrackedPoint~%float64 x~%float64 y~%float64 v~%float64 a~%float64 theta~%float64 theta_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedObjectPose>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedObjectPose>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedObjectPose
    (cl:cons ':unique_id (unique_id msg))
    (cl:cons ':pose (pose msg))
))
