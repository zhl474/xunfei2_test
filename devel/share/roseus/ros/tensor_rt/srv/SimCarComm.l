;; Auto-generated. Do not edit!


(when (boundp 'tensor_rt::SimCarComm)
  (if (not (find-package "TENSOR_RT"))
    (make-package "TENSOR_RT"))
  (shadow 'SimCarComm (find-package "TENSOR_RT")))
(unless (find-package "TENSOR_RT::SIMCARCOMM")
  (make-package "TENSOR_RT::SIMCARCOMM"))
(unless (find-package "TENSOR_RT::SIMCARCOMMREQUEST")
  (make-package "TENSOR_RT::SIMCARCOMMREQUEST"))
(unless (find-package "TENSOR_RT::SIMCARCOMMRESPONSE")
  (make-package "TENSOR_RT::SIMCARCOMMRESPONSE"))

(in-package "ROS")





(defclass tensor_rt::SimCarCommRequest
  :super ros::object
  :slots (_room _class ))

(defmethod tensor_rt::SimCarCommRequest
  (:init
   (&key
    ((:room __room) 0)
    ((:class __class) 0)
    )
   (send-super :init)
   (setq _room (round __room))
   (setq _class (round __class))
   self)
  (:room
   (&optional __room)
   (if __room (setq _room __room)) _room)
  (:class
   (&optional __class)
   (if __class (setq _class __class)) _class)
  (:serialization-length
   ()
   (+
    ;; int32 _room
    4
    ;; int32 _class
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _room
       (write-long _room s)
     ;; int32 _class
       (write-long _class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _room
     (setq _room (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _class
     (setq _class (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tensor_rt::SimCarCommResponse
  :super ros::object
  :slots (_target_class ))

(defmethod tensor_rt::SimCarCommResponse
  (:init
   (&key
    ((:target_class __target_class) 0)
    )
   (send-super :init)
   (setq _target_class (round __target_class))
   self)
  (:target_class
   (&optional __target_class)
   (if __target_class (setq _target_class __target_class)) _target_class)
  (:serialization-length
   ()
   (+
    ;; int32 _target_class
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target_class
       (write-long _target_class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target_class
     (setq _target_class (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tensor_rt::SimCarComm
  :super ros::object
  :slots ())

(setf (get tensor_rt::SimCarComm :md5sum-) "932cc26da5c945312b522ef7ce57d5c8")
(setf (get tensor_rt::SimCarComm :datatype-) "tensor_rt/SimCarComm")
(setf (get tensor_rt::SimCarComm :request) tensor_rt::SimCarCommRequest)
(setf (get tensor_rt::SimCarComm :response) tensor_rt::SimCarCommResponse)

(defmethod tensor_rt::SimCarCommRequest
  (:response () (instance tensor_rt::SimCarCommResponse :init)))

(setf (get tensor_rt::SimCarCommRequest :md5sum-) "932cc26da5c945312b522ef7ce57d5c8")
(setf (get tensor_rt::SimCarCommRequest :datatype-) "tensor_rt/SimCarCommRequest")
(setf (get tensor_rt::SimCarCommRequest :definition-)
      "int32 room   #目标所在房间  
int32 class  #目标类别
---
int32 target_class  #实际环境中的目标类别
")

(setf (get tensor_rt::SimCarCommResponse :md5sum-) "932cc26da5c945312b522ef7ce57d5c8")
(setf (get tensor_rt::SimCarCommResponse :datatype-) "tensor_rt/SimCarCommResponse")
(setf (get tensor_rt::SimCarCommResponse :definition-)
      "int32 room   #目标所在房间  
int32 class  #目标类别
---
int32 target_class  #实际环境中的目标类别
")



(provide :tensor_rt/SimCarComm "932cc26da5c945312b522ef7ce57d5c8")


