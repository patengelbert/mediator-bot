; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Person_Setup.msg.html

(cl:defclass <Person_Setup> (roslisp-msg-protocol:ros-message)
  ((Name
    :reader Name
    :initarg :Name
    :type cl:string
    :initform ""))
)

(cl:defclass Person_Setup (<Person_Setup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Person_Setup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Person_Setup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Person_Setup> is deprecated: use beginner_tutorials-msg:Person_Setup instead.")))

(cl:ensure-generic-function 'Name-val :lambda-list '(m))
(cl:defmethod Name-val ((m <Person_Setup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Name-val is deprecated.  Use beginner_tutorials-msg:Name instead.")
  (Name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Person_Setup>) ostream)
  "Serializes a message object of type '<Person_Setup>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Person_Setup>) istream)
  "Deserializes a message object of type '<Person_Setup>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Person_Setup>)))
  "Returns string type for a message object of type '<Person_Setup>"
  "beginner_tutorials/Person_Setup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Person_Setup)))
  "Returns string type for a message object of type 'Person_Setup"
  "beginner_tutorials/Person_Setup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Person_Setup>)))
  "Returns md5sum for a message object of type '<Person_Setup>"
  "cdf8d6bab384a2e7902ca6add60245eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Person_Setup)))
  "Returns md5sum for a message object of type 'Person_Setup"
  "cdf8d6bab384a2e7902ca6add60245eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Person_Setup>)))
  "Returns full string definition for message of type '<Person_Setup>"
  (cl:format cl:nil "string Name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Person_Setup)))
  "Returns full string definition for message of type 'Person_Setup"
  (cl:format cl:nil "string Name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Person_Setup>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Person_Setup>))
  "Converts a ROS message object to a list"
  (cl:list 'Person_Setup
    (cl:cons ':Name (Name msg))
))
