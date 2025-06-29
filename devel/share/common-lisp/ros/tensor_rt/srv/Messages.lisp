; Auto-generated. Do not edit!


(cl:in-package tensor_rt-srv)


;//! \htmlinclude Messages-request.msg.html

(cl:defclass <Messages-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:integer
    :initform 0))
)

(cl:defclass Messages-request (<Messages-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Messages-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Messages-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-srv:<Messages-request> is deprecated: use tensor_rt-srv:Messages-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Messages-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-srv:object-val is deprecated.  Use tensor_rt-srv:object instead.")
  (object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Messages-request>) ostream)
  "Serializes a message object of type '<Messages-request>"
  (cl:let* ((signed (cl:slot-value msg 'object)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Messages-request>) istream)
  "Deserializes a message object of type '<Messages-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Messages-request>)))
  "Returns string type for a service object of type '<Messages-request>"
  "tensor_rt/MessagesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Messages-request)))
  "Returns string type for a service object of type 'Messages-request"
  "tensor_rt/MessagesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Messages-request>)))
  "Returns md5sum for a message object of type '<Messages-request>"
  "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Messages-request)))
  "Returns md5sum for a message object of type 'Messages-request"
  "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Messages-request>)))
  "Returns full string definition for message of type '<Messages-request>"
  (cl:format cl:nil "int32 object~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Messages-request)))
  "Returns full string definition for message of type 'Messages-request"
  (cl:format cl:nil "int32 object~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Messages-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Messages-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Messages-request
    (cl:cons ':object (object msg))
))
;//! \htmlinclude Messages-response.msg.html

(cl:defclass <Messages-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Messages-response (<Messages-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Messages-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Messages-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tensor_rt-srv:<Messages-response> is deprecated: use tensor_rt-srv:Messages-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Messages-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tensor_rt-srv:result-val is deprecated.  Use tensor_rt-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Messages-response>) ostream)
  "Serializes a message object of type '<Messages-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Messages-response>) istream)
  "Deserializes a message object of type '<Messages-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'result) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'result)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Messages-response>)))
  "Returns string type for a service object of type '<Messages-response>"
  "tensor_rt/MessagesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Messages-response)))
  "Returns string type for a service object of type 'Messages-response"
  "tensor_rt/MessagesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Messages-response>)))
  "Returns md5sum for a message object of type '<Messages-response>"
  "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Messages-response)))
  "Returns md5sum for a message object of type 'Messages-response"
  "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Messages-response>)))
  "Returns full string definition for message of type '<Messages-response>"
  (cl:format cl:nil "int32[] result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Messages-response)))
  "Returns full string definition for message of type 'Messages-response"
  (cl:format cl:nil "int32[] result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Messages-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'result) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Messages-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Messages-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Messages)))
  'Messages-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Messages)))
  'Messages-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Messages)))
  "Returns string type for a service object of type '<Messages>"
  "tensor_rt/Messages")