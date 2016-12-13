
(cl:in-package :asdf)

(defsystem "beginner_tutorials-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StartEnrollment" :depends-on ("_package_StartEnrollment"))
    (:file "_package_StartEnrollment" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "EndEnrollment" :depends-on ("_package_EndEnrollment"))
    (:file "_package_EndEnrollment" :depends-on ("_package"))
    (:file "endEnrollment" :depends-on ("_package_endEnrollment"))
    (:file "_package_endEnrollment" :depends-on ("_package"))
    (:file "MedBotSpeechQuery" :depends-on ("_package_MedBotSpeechQuery"))
    (:file "_package_MedBotSpeechQuery" :depends-on ("_package"))
  ))