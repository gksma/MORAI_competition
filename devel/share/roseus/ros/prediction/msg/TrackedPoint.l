;; Auto-generated. Do not edit!


(when (boundp 'prediction::TrackedPoint)
  (if (not (find-package "PREDICTION"))
    (make-package "PREDICTION"))
  (shadow 'TrackedPoint (find-package "PREDICTION")))
(unless (find-package "PREDICTION::TRACKEDPOINT")
  (make-package "PREDICTION::TRACKEDPOINT"))

(in-package "ROS")
;;//! \htmlinclude TrackedPoint.msg.html


(defclass prediction::TrackedPoint
  :super ros::object
  :slots (_x _y _v _a _theta _theta_rate ))

(defmethod prediction::TrackedPoint
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:v __v) 0.0)
    ((:a __a) 0.0)
    ((:theta __theta) 0.0)
    ((:theta_rate __theta_rate) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _v (float __v))
   (setq _a (float __a))
   (setq _theta (float __theta))
   (setq _theta_rate (float __theta_rate))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:theta_rate
   (&optional __theta_rate)
   (if __theta_rate (setq _theta_rate __theta_rate)) _theta_rate)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _v
    8
    ;; float64 _a
    8
    ;; float64 _theta
    8
    ;; float64 _theta_rate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta_rate
       (sys::poke _theta_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta_rate
     (setq _theta_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get prediction::TrackedPoint :md5sum-) "250909336024d809e043543daf5e70da")
(setf (get prediction::TrackedPoint :datatype-) "prediction/TrackedPoint")
(setf (get prediction::TrackedPoint :definition-)
      "float64 x
float64 y
float64 v
float64 a
float64 theta
float64 theta_rate

")



(provide :prediction/TrackedPoint "250909336024d809e043543daf5e70da")


