
(cl:in-package :asdf)

(defsystem "robocon_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LaserScan" :depends-on ("_package_LaserScan"))
    (:file "_package_LaserScan" :depends-on ("_package"))
    (:file "chassis_msg" :depends-on ("_package_chassis_msg"))
    (:file "_package_chassis_msg" :depends-on ("_package"))
    (:file "commonMsg" :depends-on ("_package_commonMsg"))
    (:file "_package_commonMsg" :depends-on ("_package"))
    (:file "pwmset" :depends-on ("_package_pwmset"))
    (:file "_package_pwmset" :depends-on ("_package"))
    (:file "remoter_msg" :depends-on ("_package_remoter_msg"))
    (:file "_package_remoter_msg" :depends-on ("_package"))
  ))