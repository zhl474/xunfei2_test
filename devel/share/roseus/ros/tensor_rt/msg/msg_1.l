;; Auto-generated. Do not edit!


(when (boundp 'tensor_rt::msg_1)
  (if (not (find-package "TENSOR_RT"))
    (make-package "TENSOR_RT"))
  (shadow 'msg_1 (find-package "TENSOR_RT")))
(unless (find-package "TENSOR_RT::MSG_1")
  (make-package "TENSOR_RT::MSG_1"))

(in-package "ROS")
;;//! \htmlinclude msg_1.msg.html


(defclass tensor_rt::msg_1
  :super ros::object
  :slots (_target_class ))

(defmethod tensor_rt::msg_1
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
    ;; uint16 _target_class
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _target_class
       (write-word _target_class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _target_class
     (setq _target_class (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get tensor_rt::msg_1 :md5sum-) "997878a101ab058a18257fccc8baf249")
(setf (get tensor_rt::msg_1 :datatype-) "tensor_rt/msg_1")
(setf (get tensor_rt::msg_1 :definition-)
      "uint16 target_class

")



(provide :tensor_rt/msg_1 "997878a101ab058a18257fccc8baf249")


