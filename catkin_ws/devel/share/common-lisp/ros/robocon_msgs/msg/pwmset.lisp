; Auto-generated. Do not edit!


(cl:in-package robocon_msgs-msg)


;//! \htmlinclude pwmset.msg.html

(cl:defclass <pwmset> (roslisp-msg-protocol:ros-message)
  ((pwm
    :reader pwm
    :initarg :pwm
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass pwmset (<pwmset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pwmset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pwmset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robocon_msgs-msg:<pwmset> is deprecated: use robocon_msgs-msg:pwmset instead.")))

(cl:ensure-generic-function 'pwm-val :lambda-list '(m))
(cl:defmethod pwm-val ((m <pwmset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robocon_msgs-msg:pwm-val is deprecated.  Use robocon_msgs-msg:pwm instead.")
  (pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pwmset>) ostream)
  "Serializes a message object of type '<pwmset>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pwm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pwm))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pwmset>) istream)
  "Deserializes a message object of type '<pwmset>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pwm) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pwm)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pwmset>)))
  "Returns string type for a message object of type '<pwmset>"
  "robocon_msgs/pwmset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pwmset)))
  "Returns string type for a message object of type 'pwmset"
  "robocon_msgs/pwmset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pwmset>)))
  "Returns md5sum for a message object of type '<pwmset>"
  "d8d84643029731b92d628f398337d291")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pwmset)))
  "Returns md5sum for a message object of type 'pwmset"
  "d8d84643029731b92d628f398337d291")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pwmset>)))
  "Returns full string definition for message of type '<pwmset>"
  (cl:format cl:nil "float32[] pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pwmset)))
  "Returns full string definition for message of type 'pwmset"
  (cl:format cl:nil "float32[] pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pwmset>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pwm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pwmset>))
  "Converts a ROS message object to a list"
  (cl:list 'pwmset
    (cl:cons ':pwm (pwm msg))
))
