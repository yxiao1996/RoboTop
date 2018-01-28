;; Auto-generated. Do not edit!


(when (boundp 'robocon_msgs::chassis_msg)
  (if (not (find-package "ROBOCON_MSGS"))
    (make-package "ROBOCON_MSGS"))
  (shadow 'chassis_msg (find-package "ROBOCON_MSGS")))
(unless (find-package "ROBOCON_MSGS::CHASSIS_MSG")
  (make-package "ROBOCON_MSGS::CHASSIS_MSG"))

(in-package "ROS")
;;//! \htmlinclude chassis_msg.msg.html


(defclass robocon_msgs::chassis_msg
  :super ros::object
  :slots (_chassis _str ))

(defmethod robocon_msgs::chassis_msg
  (:init
   (&key
    ((:chassis __chassis) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:str __str) "")
    )
   (send-super :init)
   (setq _chassis __chassis)
   (setq _str (string __str))
   self)
  (:chassis
   (&optional __chassis)
   (if __chassis (setq _chassis __chassis)) _chassis)
  (:str
   (&optional __str)
   (if __str (setq _str __str)) _str)
  (:serialization-length
   ()
   (+
    ;; float64[] _chassis
    (* 8    (length _chassis)) 4
    ;; string _str
    4 (length _str)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _chassis
     (write-long (length _chassis) s)
     (dotimes (i (length _chassis))
       (sys::poke (elt _chassis i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _str
       (write-long (length _str) s) (princ _str s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _chassis
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _chassis (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _chassis i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _str
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _str (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robocon_msgs::chassis_msg :md5sum-) "a899e5eecd9f022d7c1366dd45f69c78")
(setf (get robocon_msgs::chassis_msg :datatype-) "robocon_msgs/chassis_msg")
(setf (get robocon_msgs::chassis_msg :definition-)
      "float64[] chassis
string str

")



(provide :robocon_msgs/chassis_msg "a899e5eecd9f022d7c1366dd45f69c78")


