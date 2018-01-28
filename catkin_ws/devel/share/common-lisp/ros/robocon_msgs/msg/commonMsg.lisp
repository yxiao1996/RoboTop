; Auto-generated. Do not edit!


(cl:in-package robocon_msgs-msg)


;//! \htmlinclude commonMsg.msg.html

(cl:defclass <commonMsg> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass commonMsg (<commonMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commonMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commonMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robocon_msgs-msg:<commonMsg> is deprecated: use robocon_msgs-msg:commonMsg instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <commonMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robocon_msgs-msg:data-val is deprecated.  Use robocon_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commonMsg>) ostream)
  "Serializes a message object of type '<commonMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commonMsg>) istream)
  "Deserializes a message object of type '<commonMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commonMsg>)))
  "Returns string type for a message object of type '<commonMsg>"
  "robocon_msgs/commonMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commonMsg)))
  "Returns string type for a message object of type 'commonMsg"
  "robocon_msgs/commonMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commonMsg>)))
  "Returns md5sum for a message object of type '<commonMsg>"
  "ac9c931aaf6ce145ea0383362e83c70b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commonMsg)))
  "Returns md5sum for a message object of type 'commonMsg"
  "ac9c931aaf6ce145ea0383362e83c70b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commonMsg>)))
  "Returns full string definition for message of type '<commonMsg>"
  (cl:format cl:nil "int8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commonMsg)))
  "Returns full string definition for message of type 'commonMsg"
  (cl:format cl:nil "int8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commonMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commonMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'commonMsg
    (cl:cons ':data (data msg))
))
