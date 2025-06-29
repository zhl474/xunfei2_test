;; Auto-generated. Do not edit!


(when (boundp 'tensor_rt::Messages)
  (if (not (find-package "TENSOR_RT"))
    (make-package "TENSOR_RT"))
  (shadow 'Messages (find-package "TENSOR_RT")))
(unless (find-package "TENSOR_RT::MESSAGES")
  (make-package "TENSOR_RT::MESSAGES"))
(unless (find-package "TENSOR_RT::MESSAGESREQUEST")
  (make-package "TENSOR_RT::MESSAGESREQUEST"))
(unless (find-package "TENSOR_RT::MESSAGESRESPONSE")
  (make-package "TENSOR_RT::MESSAGESRESPONSE"))

(in-package "ROS")





(defclass tensor_rt::MessagesRequest
  :super ros::object
  :slots (_object ))

(defmethod tensor_rt::MessagesRequest
  (:init
   (&key
    ((:object __object) 0)
    )
   (send-super :init)
   (setq _object (round __object))
   self)
  (:object
   (&optional __object)
   (if __object (setq _object __object)) _object)
  (:serialization-length
   ()
   (+
    ;; int32 _object
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _object
       (write-long _object s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _object
     (setq _object (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tensor_rt::MessagesResponse
  :super ros::object
  :slots (_result ))

(defmethod tensor_rt::MessagesResponse
  (:init
   (&key
    ((:result __result) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32[] _result
    (* 4    (length _result)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _result
     (write-long (length _result) s)
     (dotimes (i (length _result))
       (write-long (elt _result i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _result
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _result (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _result i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass tensor_rt::Messages
  :super ros::object
  :slots ())

(setf (get tensor_rt::Messages :md5sum-) "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(setf (get tensor_rt::Messages :datatype-) "tensor_rt/Messages")
(setf (get tensor_rt::Messages :request) tensor_rt::MessagesRequest)
(setf (get tensor_rt::Messages :response) tensor_rt::MessagesResponse)

(defmethod tensor_rt::MessagesRequest
  (:response () (instance tensor_rt::MessagesResponse :init)))

(setf (get tensor_rt::MessagesRequest :md5sum-) "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(setf (get tensor_rt::MessagesRequest :datatype-) "tensor_rt/MessagesRequest")
(setf (get tensor_rt::MessagesRequest :definition-)
      "int32 object

---
int32[] result
")

(setf (get tensor_rt::MessagesResponse :md5sum-) "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")
(setf (get tensor_rt::MessagesResponse :datatype-) "tensor_rt/MessagesResponse")
(setf (get tensor_rt::MessagesResponse :definition-)
      "int32 object

---
int32[] result
")



(provide :tensor_rt/Messages "bdb4ecf8e91e5c89bb2f7e7af5cf8ea3")


