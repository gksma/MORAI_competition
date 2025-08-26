;; Auto-generated. Do not edit!


(when (boundp 'prediction::TrackedObjectPose)
  (if (not (find-package "PREDICTION"))
    (make-package "PREDICTION"))
  (shadow 'TrackedObjectPose (find-package "PREDICTION")))
(unless (find-package "PREDICTION::TRACKEDOBJECTPOSE")
  (make-package "PREDICTION::TRACKEDOBJECTPOSE"))

(in-package "ROS")
;;//! \htmlinclude TrackedObjectPose.msg.html


(defclass prediction::TrackedObjectPose
  :super ros::object
  :slots (_unique_id _pose ))

(defmethod prediction::TrackedObjectPose
  (:init
   (&key
    ((:unique_id __unique_id) 0)
    ((:pose __pose) (instance prediction::TrackedPoint :init))
    )
   (send-super :init)
   (setq _unique_id (round __unique_id))
   (setq _pose __pose)
   self)
  (:unique_id
   (&optional __unique_id)
   (if __unique_id (setq _unique_id __unique_id)) _unique_id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; int32 _unique_id
    4
    ;; prediction/TrackedPoint _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _unique_id
       (write-long _unique_id s)
     ;; prediction/TrackedPoint _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _unique_id
     (setq _unique_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; prediction/TrackedPoint _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get prediction::TrackedObjectPose :md5sum-) "7690af574bf077cf4e6a344bb466312f")
(setf (get prediction::TrackedObjectPose :datatype-) "prediction/TrackedObjectPose")
(setf (get prediction::TrackedObjectPose :definition-)
      "int32 unique_id

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



(provide :prediction/TrackedObjectPose "7690af574bf077cf4e6a344bb466312f")


