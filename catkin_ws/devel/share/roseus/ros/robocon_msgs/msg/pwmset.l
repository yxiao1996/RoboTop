;; Auto-generated. Do not edit!


(when (boundp 'robocon_msgs::pwmset)
  (if (not (find-package "ROBOCON_MSGS"))
    (make-package "ROBOCON_MSGS"))
  (shadow 'pwmset (find-package "ROBOCON_MSGS")))
(unless (find-package "ROBOCON_MSGS::PWMSET")
  (make-package "ROBOCON_MSGS::PWMSET"))

(in-package "ROS")
;;//! \htmlinclude pwmset.msg.html


(defclass robocon_msgs::pwmset
  :super ros::object
  :slots (_pwm ))

(defmethod robocon_msgs::pwmset
  (:init
   (&key
    ((:pwm __pwm) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pwm __pwm)
   self)
  (:pwm
   (&optional __pwm)
   (if __pwm (setq _pwm __pwm)) _pwm)
  (:serialization-length
   ()
   (+
    ;; float32[] _pwm
    (* 4    (length _pwm)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _pwm
     (write-long (length _pwm) s)
     (dotimes (i (length _pwm))
       (sys::poke (elt _pwm i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _pwm
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pwm (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pwm i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get robocon_msgs::pwmset :md5sum-) "d8d84643029731b92d628f398337d291")
(setf (get robocon_msgs::pwmset :datatype-) "robocon_msgs/pwmset")
(setf (get robocon_msgs::pwmset :definition-)
      "float32[] pwm

")



(provide :robocon_msgs/pwmset "d8d84643029731b92d628f398337d291")


