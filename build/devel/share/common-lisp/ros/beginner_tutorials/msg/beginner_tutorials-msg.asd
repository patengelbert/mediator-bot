
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Percentage" :depends-on ("_package_Percentage"))
    (:file "_package_Percentage" :depends-on ("_package"))
    (:file "Person_Setup" :depends-on ("_package_Person_Setup"))
    (:file "_package_Person_Setup" :depends-on ("_package"))
    (:file "Transcript" :depends-on ("_package_Transcript"))
    (:file "_package_Transcript" :depends-on ("_package"))
    (:file "Person_Register" :depends-on ("_package_Person_Register"))
    (:file "_package_Person_Register" :depends-on ("_package"))
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))