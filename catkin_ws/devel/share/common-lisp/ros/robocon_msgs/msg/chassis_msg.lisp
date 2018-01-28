; Auto-generated. Do not edit!


(cl:in-package robocon_msgs-msg)


;//! \htmlinclude chassis_msg.msg.html

(cl:defclass <chassis_msg> (roslisp-msg-protocol:ros-message)
  ((chassis
    :reader chassis
    :initarg :chassis
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass chassis_msg (<chassis_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chassis_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chassis_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robocon_msgs-msg:<chassis_msg> is deprecated: use robocon_msgs-msg:chassis_msg instead.")))

(cl:ensure-generic-function 'chassis-val :lambda-list '(m))
(cl:defmethod chassis-val ((m <chassis_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robocon_msgs-msg:chassis-val is deprecated.  Use robocon_msgs-msg:chassis instead.")
  (chassis m))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <chassis_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robocon_msgs-msg:str-val is deprecated.  Use robocon_msgs-msg:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chassis_msg>) ostream)
  "Serializes a message object of type '<chassis_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'chassis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'chassis))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chassis_msg>) istream)
  "Deserializes a message object of type '<chassis_msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'chassis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'chassis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chassis_msg>)))
  "Returns string type for a message object of type '<chassis_msg>"
  "robocon_msgs/chassis_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chassis_msg)))
  "Returns string type for a message object of type 'chassis_msg"
  "robocon_msgs/chassis_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chassis_msg>)))
  "Returns md5sum for a message object of type '<chassis_msg>"
  "a899e5eecd9f022d7c1366dd45f69c78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chassis_msg)))
  "Returns md5sum for a message object of type 'chassis_msg"
  "a899e5eecd9f022d7c1366dd45f69c78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chassis_msg>)))
  "Returns full string definition for message of type '<chassis_msg>"
  (cl:format cl:nil "float64[] chassis~%string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chassis_msg)))
  "Returns full string definition for message of type 'chassis_msg"
  (cl:format cl:nil "float64[] chassis~%string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chassis_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'chassis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chassis_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'chassis_msg
    (cl:cons ':chassis (chassis msg))
    (cl:cons ':str (str msg))
))
