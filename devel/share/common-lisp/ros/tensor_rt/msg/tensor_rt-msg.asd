
(cl:in-package :asdf)

(defsystem "tensor_rt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_1" :depends-on ("_package_msg_1"))
    (:file "_package_msg_1" :depends-on ("_package"))
    (:file "msg_2" :depends-on ("_package_msg_2"))
    (:file "_package_msg_2" :depends-on ("_package"))
  ))