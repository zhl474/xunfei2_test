; Auto-generated. Do not edit!


(cl:in-package tensor_rt-srv)


;//! \htmlinclude SimCarComm-request.msg.html

(cl:defclass <SimCarComm-request> (roslisp-msg-protocol:ros-message)
  ((room
    :reader room
    :initarg :room
    :type cl:integer
    :initform 0)
   (class
    :reader class
    :initarg :class
    :type cl:integer
    :initform 0))
)

(cl:defclass SimCarComm-request (<SimCarComm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimCarComm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimCarComm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-srv:<SimCarComm-request> is deprecated: use tensor_rt-srv:SimCarComm-request instead.")))

(cl:ensure-generic-function 'room-val :lambda-list '(m))
(cl:defmethod room-val ((m <SimCarComm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-srv:room-val is deprecated.  Use tensor_rt-srv:room instead.")
  (room m))

(cl:ensure-generic-function 'class-val :lambda-list '(m))
(cl:defmethod class-val ((m <SimCarComm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-srv:class-val is deprecated.  Use tensor_rt-srv:class instead.")
  (class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimCarComm-request>) ostream)
  "Serializes a message object of type '<SimCarComm-request>"
  (cl:let* ((signed (cl:slot-value msg 'room)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'class)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimCarComm-request>) istream)
  "Deserializes a message object of type '<SimCarComm-request>"
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
      (cl:setf (cl:slot-value msg 'class) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimCarComm-request>)))
  "Returns string type for a service object of type '<SimCarComm-request>"
  "tensor_rt/SimCarCommRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimCarComm-request)))
  "Returns string type for a service object of type 'SimCarComm-request"
  "tensor_rt/SimCarCommRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimCarComm-request>)))
  "Returns md5sum for a message object of type '<SimCarComm-request>"
  "932cc26da5c945312b522ef7ce57d5c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimCarComm-request)))
  "Returns md5sum for a message object of type 'SimCarComm-request"
  "932cc26da5c945312b522ef7ce57d5c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimCarComm-request>)))
  "Returns full string definition for message of type '<SimCarComm-request>"
  (cl:format cl:nil "int32 room   #目标所在房间  ~%int32 class  #目标类别~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimCarComm-request)))
  "Returns full string definition for message of type 'SimCarComm-request"
  (cl:format cl:nil "int32 room   #目标所在房间  ~%int32 class  #目标类别~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimCarComm-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimCarComm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SimCarComm-request
    (cl:cons ':room (room msg))
    (cl:cons ':class (class msg))
))
;//! \htmlinclude SimCarComm-response.msg.html

(cl:defclass <SimCarComm-response> (roslisp-msg-protocol:ros-message)
  ((target_class
    :reader target_class
    :initarg :target_class
    :type cl:integer
    :initform 0))
)

(cl:defclass SimCarComm-response (<SimCarComm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimCarComm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimCarComm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-srv:<SimCarComm-response> is deprecated: use tensor_rt-srv:SimCarComm-response instead.")))

(cl:ensure-generic-function 'target_class-val :lambda-list '(m))
(cl:defmethod target_class-val ((m <SimCarComm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-srv:target_class-val is deprecated.  Use tensor_rt-srv:target_class instead.")
  (target_class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimCarComm-response>) ostream)
  "Serializes a message object of type '<SimCarComm-response>"
  (cl:let* ((signed (cl:slot-value msg 'target_class)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimCarComm-response>) istream)
  "Deserializes a message object of type '<SimCarComm-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_class) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimCarComm-response>)))
  "Returns string type for a service object of type '<SimCarComm-response>"
  "tensor_rt/SimCarCommResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimCarComm-response)))
  "Returns string type for a service object of type 'SimCarComm-response"
  "tensor_rt/SimCarCommResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimCarComm-response>)))
  "Returns md5sum for a message object of type '<SimCarComm-response>"
  "932cc26da5c945312b522ef7ce57d5c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimCarComm-response)))
  "Returns md5sum for a message object of type 'SimCarComm-response"
  "932cc26da5c945312b522ef7ce57d5c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimCarComm-response>)))
  "Returns full string definition for message of type '<SimCarComm-response>"
  (cl:format cl:nil "int32 target_class  #实际环境中的目标类别~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimCarComm-response)))
  "Returns full string definition for message of type 'SimCarComm-response"
  (cl:format cl:nil "int32 target_class  #实际环境中的目标类别~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimCarComm-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimCarComm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SimCarComm-response
    (cl:cons ':target_class (target_class msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SimCarComm)))
  'SimCarComm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SimCarComm)))
  'SimCarComm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimCarComm)))
  "Returns string type for a service object of type '<SimCarComm>"
  "tensor_rt/SimCarComm")