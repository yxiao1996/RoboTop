;; Auto-generated. Do not edit!


(when (boundp 'robocon_msgs::commonMsg)
  (if (not (find-package "ROBOCON_MSGS"))
    (make-package "ROBOCON_MSGS"))
  (shadow 'commonMsg (find-package "ROBOCON_MSGS")))
(unless (find-package "ROBOCON_MSGS::COMMONMSG")
  (make-package "ROBOCON_MSGS::COMMONMSG"))

(in-package "ROS")
;;//! \htmlinclude commonMsg.msg.html


(defclass robocon_msgs::commonMsg
  :super ros::object
  :slots (_data ))

(defmethod robocon_msgs::commonMsg
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-byte (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _data i) 127) (setf (elt _data i) (- (elt _data i) 256)))
     ))
   ;;
   self)
  )

(setf (get robocon_msgs::commonMsg :md5sum-) "ac9c931aaf6ce145ea0383362e83c70b")
(setf (get robocon_msgs::commonMsg :datatype-) "robocon_msgs/commonMsg")
(setf (get robocon_msgs::commonMsg :definition-)
      "int8[] data

")



(provide :robocon_msgs/commonMsg "ac9c931aaf6ce145ea0383362e83c70b")


