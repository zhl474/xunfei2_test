; Auto-generated. Do not edit!


(cl:in-package tensor_rt-msg)


;//! \htmlinclude msg_2.msg.html

(cl:defclass <msg_2> (roslisp-msg-protocol:ros-message)
  ((room
    :reader room
    :initarg :room
    :type cl:integer
    :initform 0)
   (detected_class
    :reader detected_class
    :initarg :detected_class
    :type cl:integer
    :initform 0))
)

(cl:defclass msg_2 (<msg_2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-msg:<msg_2> is deprecated: use tensor_rt-msg:msg_2 instead.")))

(cl:ensure-generic-function 'room-val :lambda-list '(m))
(cl:defmethod room-val ((m <msg_2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-msg:room-val is deprecated.  Use tensor_rt-msg:room instead.")
  (room m))

(cl:ensure-generic-function 'detected_class-val :lambda-list '(m))
(cl:defmethod detected_class-val ((m <msg_2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-msg:detected_class-val is deprecated.  Use tensor_rt-msg:detected_class instead.")
  (detected_class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_2>) ostream)
  "Serializes a message object of type '<msg_2>"
  (cl:let* ((signed (cl:slot-value msg 'room)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'detected_class)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_2>) istream)
  "Deserializes a message object of type '<msg_2>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'room) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'detected_class) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_2>)))
  "Returns string type for a message object of type '<msg_2>"
  "tensor_rt/msg_2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_2)))
  "Returns string type for a message object of type 'msg_2"
  "tensor_rt/msg_2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_2>)))
  "Returns md5sum for a message object of type '<msg_2>"
  "d4962e33f389591ae16435b9eda70efd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_2)))
  "Returns md5sum for a message object of type 'msg_2"
  "d4962e33f389591ae16435b9eda70efd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_2>)))
  "Returns full string definition for message of type '<msg_2>"
  (cl:format cl:nil "int32 room~%int32 detected_class~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_2)))
  "Returns full string definition for message of type 'msg_2"
  (cl:format cl:nil "int32 room~%int32 detected_class~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_2>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_2>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_2
    (cl:cons ':room (room msg))
    (cl:cons ':detected_class (detected_class msg))
))
