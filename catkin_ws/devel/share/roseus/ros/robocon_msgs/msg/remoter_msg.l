;; Auto-generated. Do not edit!


(when (boundp 'robocon_msgs::remoter_msg)
  (if (not (find-package "ROBOCON_MSGS"))
    (make-package "ROBOCON_MSGS"))
  (shadow 'remoter_msg (find-package "ROBOCON_MSGS")))
(unless (find-package "ROBOCON_MSGS::REMOTER_MSG")
  (make-package "ROBOCON_MSGS::REMOTER_MSG"))

(in-package "ROS")
;;//! \htmlinclude remoter_msg.msg.html


(defclass robocon_msgs::remoter_msg
  :super ros::object
  :slots (_position _remote_data _attitude ))

(defmethod robocon_msgs::remoter_msg
  (:init
   (&key
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remote_data __remote_data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:attitude __attitude) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _position __position)
   (setq _remote_data __remote_data)
   (setq _attitude __attitude)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:remote_data
   (&optional __remote_data)
   (if __remote_data (setq _remote_data __remote_data)) _remote_data)
  (:attitude
   (&optional __attitude)
   (if __attitude (setq _attitude __attitude)) _attitude)
  (:serialization-length
   ()
   (+
    ;; float32[] _position
    (* 4    (length _position)) 4
    ;; float32[] _remote_data
    (* 4    (length _remote_data)) 4
    ;; float64[] _attitude
    (* 8    (length _attitude)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remote_data
     (write-long (length _remote_data) s)
     (dotimes (i (length _remote_data))
       (sys::poke (elt _remote_data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float64[] _attitude
     (write-long (length _attitude) s)
     (dotimes (i (length _attitude))
       (sys::poke (elt _attitude i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remote_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remote_data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remote_data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float64[] _attitude
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _attitude (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _attitude i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robocon_msgs::remoter_msg :md5sum-) "75585e5d82b120054891679ec4f80561")
(setf (get robocon_msgs::remoter_msg :datatype-) "robocon_msgs/remoter_msg")
(setf (get robocon_msgs::remoter_msg :definition-)
      "float32[] position
float32[] remote_data
float64[] attitude

")



(provide :robocon_msgs/remoter_msg "75585e5d82b120054891679ec4f80561")


