; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Person_Register.msg.html

(cl:defclass <Person_Register> (roslisp-msg-protocol:ros-message)
  ((Person
    :reader Person
    :initarg :Person
    :type cl:string
    :initform "")
   (Success
    :reader Success
    :initarg :Success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Person_Register (<Person_Register>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Person_Register>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Person_Register)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Person_Register> is deprecated: use beginner_tutorials-msg:Person_Register instead.")))

(cl:ensure-generic-function 'Person-val :lambda-list '(m))
(cl:defmethod Person-val ((m <Person_Register>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Person-val is deprecated.  Use beginner_tutorials-msg:Person instead.")
  (Person m))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <Person_Register>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Success-val is deprecated.  Use beginner_tutorials-msg:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Person_Register>) ostream)
  "Serializes a message object of type '<Person_Register>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Person))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Person))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Person_Register>) istream)
  "Deserializes a message object of type '<Person_Register>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Person) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Person) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'Success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Person_Register>)))
  "Returns string type for a message object of type '<Person_Register>"
  "beginner_tutorials/Person_Register")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Person_Register)))
  "Returns string type for a message object of type 'Person_Register"
  "beginner_tutorials/Person_Register")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Person_Register>)))
  "Returns md5sum for a message object of type '<Person_Register>"
  "61d32fc4e5a7b7b9337763e7c1fb476e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Person_Register)))
  "Returns md5sum for a message object of type 'Person_Register"
  "61d32fc4e5a7b7b9337763e7c1fb476e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Person_Register>)))
  "Returns full string definition for message of type '<Person_Register>"
  (cl:format cl:nil "string Person~%bool Success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Person_Register)))
  "Returns full string definition for message of type 'Person_Register"
  (cl:format cl:nil "string Person~%bool Success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Person_Register>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Person))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Person_Register>))
  "Converts a ROS message object to a list"
  (cl:list 'Person_Register
    (cl:cons ':Person (Person msg))
    (cl:cons ':Success (Success msg))
))
