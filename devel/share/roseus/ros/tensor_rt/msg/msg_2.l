;; Auto-generated. Do not edit!


(when (boundp 'tensor_rt::msg_2)
  (if (not (find-package "TENSOR_RT"))
    (make-package "TENSOR_RT"))
  (shadow 'msg_2 (find-package "TENSOR_RT")))
(unless (find-package "TENSOR_RT::MSG_2")
  (make-package "TENSOR_RT::MSG_2"))

(in-package "ROS")
;;//! \htmlinclude msg_2.msg.html


(defclass tensor_rt::msg_2
  :super ros::object
  :slots (_room _detected_class ))

(defmethod tensor_rt::msg_2
  (:init
   (&key
    ((:room __room) 0)
    ((:detected_class __detected_class) 0)
    )
   (send-super :init)
   (setq _room (round __room))
   (setq _detected_class (round __detected_class))
   self)
  (:room
   (&optional __room)
   (if __room (setq _room __room)) _room)
  (:detected_class
   (&optional __detected_class)
   (if __detected_class (setq _detected_class __detected_class)) _detected_class)
  (:serialization-length
   ()
   (+
    ;; int32 _room
    4
    ;; int32 _detected_class
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _room
       (write-long _room s)
     ;; int32 _detected_class
       (write-long _detected_class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _room
     (setq _room (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _detected_class
     (setq _detected_class (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tensor_rt::msg_2 :md5sum-) "d4962e33f389591ae16435b9eda70efd")
(setf (get tensor_rt::msg_2 :datatype-) "tensor_rt/msg_2")
(setf (get tensor_rt::msg_2 :definition-)
      "int32 room
int32 detected_class
")



(provide :tensor_rt/msg_2 "d4962e33f389591ae16435b9eda70efd")


