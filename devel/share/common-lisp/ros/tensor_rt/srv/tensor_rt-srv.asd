
(cl:in-package :asdf)

(defsystem "tensor_rt-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Messages" :depends-on ("_package_Messages"))
    (:file "_package_Messages" :depends-on ("_package"))
    (:file "SimCarComm" :depends-on ("_package_SimCarComm"))
    (:file "_package_SimCarComm" :depends-on ("_package"))
  ))