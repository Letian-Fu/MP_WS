
(cl:in-package :asdf)

(defsystem "gp_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBoxArray" :depends-on ("_package_BoundingBoxArray"))
    (:file "_package_BoundingBoxArray" :depends-on ("_package"))
  ))