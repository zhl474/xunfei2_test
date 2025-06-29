; Auto-generated. Do not edit!


(cl:in-package tensor_rt-msg)


;//! \htmlinclude msg_1.msg.html

(cl:defclass <msg_1> (roslisp-msg-protocol:ros-message)
  ((target_class
    :reader target_class
    :initarg :target_class
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_1 (<msg_1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-msg:<msg_1> is deprecated: use tensor_rt-msg:msg_1 instead.")))

(cl:ensure-generic-function 'target_class-val :lambda-list '(m))
(cl:defmethod target_class-val ((m <msg_1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-msg:target_class-val is deprecated.  Use tensor_rt-msg:target_class instead.")
  (target_class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_1>) ostream)
  "Serializes a message object of type '<msg_1>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_class)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_1>) istream)
  "Deserializes a message object of type '<msg_1>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_class)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_1>)))
  "Returns string type for a message object of type '<msg_1>"
  "tensor_rt/msg_1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_1)))
  "Returns string type for a message object of type 'msg_1"
  "tensor_rt/msg_1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_1>)))
  "Returns md5sum for a message object of type '<msg_1>"
  "997878a101ab058a18257fccc8baf249")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_1)))
  "Returns md5sum for a message object of type 'msg_1"
  "997878a101ab058a18257fccc8baf249")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_1>)))
  "Returns full string definition for message of type '<msg_1>"
  (cl:format cl:nil "uint16 target_class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_1)))
  "Returns full string definition for message of type 'msg_1"
  (cl:format cl:nil "uint16 target_class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_1>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_1>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_1
    (cl:cons ':target_class (target_class msg))
))
