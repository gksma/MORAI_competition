;; Auto-generated. Do not edit!


(when (boundp 'prediction::PredictedObjectPath)
  (if (not (find-package "PREDICTION"))
    (make-package "PREDICTION"))
  (shadow 'PredictedObjectPath (find-package "PREDICTION")))
(unless (find-package "PREDICTION::PREDICTEDOBJECTPATH")
  (make-package "PREDICTION::PREDICTEDOBJECTPATH"))

(in-package "ROS")
;;//! \htmlinclude PredictedObjectPath.msg.html


(defclass prediction::PredictedObjectPath
  :super ros::object
  :slots (_unique_id _path ))

(defmethod prediction::PredictedObjectPath
  (:init
   (&key
    ((:unique_id __unique_id) 0)
    ((:path __path) ())
    )
   (send-super :init)
   (setq _unique_id (round __unique_id))
   (setq _path __path)
   self)
  (:unique_id
   (&optional __unique_id)
   (if __unique_id (setq _unique_id __unique_id)) _unique_id)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; int32 _unique_id
    4
    ;; prediction/TrackedPoint[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _unique_id
       (write-long _unique_id s)
     ;; prediction/TrackedPoint[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _unique_id
     (setq _unique_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; prediction/TrackedPoint[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance prediction::TrackedPoint :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get prediction::PredictedObjectPath :md5sum-) "a136606ffb64a5c19dfc192304ca3b46")
(setf (get prediction::PredictedObjectPath :datatype-) "prediction/PredictedObjectPath")
(setf (get prediction::PredictedObjectPath :definition-)
      "int32 unique_id

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



(provide :prediction/PredictedObjectPath "a136606ffb64a5c19dfc192304ca3b46")


