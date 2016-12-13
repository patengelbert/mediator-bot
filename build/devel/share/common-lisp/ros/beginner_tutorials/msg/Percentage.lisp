; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Percentage.msg.html

(cl:defclass <Percentage> (roslisp-msg-protocol:ros-message)
  ((Person
    :reader Person
    :initarg :Person
    :type cl:string
    :initform "")
   (Percentage
    :reader Percentage
    :initarg :Percentage
    :type cl:integer
    :initform 0))
)

(cl:defclass Percentage (<Percentage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Percentage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Percentage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Percentage> is deprecated: use beginner_tutorials-msg:Percentage instead.")))

(cl:ensure-generic-function 'Person-val :lambda-list '(m))
(cl:defmethod Person-val ((m <Percentage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Person-val is deprecated.  Use beginner_tutorials-msg:Person instead.")
  (Person m))

(cl:ensure-generic-function 'Percentage-val :lambda-list '(m))
(cl:defmethod Percentage-val ((m <Percentage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Percentage-val is deprecated.  Use beginner_tutorials-msg:Percentage instead.")
  (Percentage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Percentage>) ostream)
  "Serializes a message object of type '<Percentage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Person))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Person))
  (cl:let* ((signed (cl:slot-value msg 'Percentage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Percentage>) istream)
  "Deserializes a message object of type '<Percentage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Person) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Person) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Percentage) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Percentage>)))
  "Returns string type for a message object of type '<Percentage>"
  "beginner_tutorials/Percentage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Percentage)))
  "Returns string type for a message object of type 'Percentage"
  "beginner_tutorials/Percentage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Percentage>)))
  "Returns md5sum for a message object of type '<Percentage>"
  "eaa025e53af943b3b327910dda9fe9c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Percentage)))
  "Returns md5sum for a message object of type 'Percentage"
  "eaa025e53af943b3b327910dda9fe9c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Percentage>)))
  "Returns full string definition for message of type '<Percentage>"
  (cl:format cl:nil "string Person~%int32 Percentage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Percentage)))
  "Returns full string definition for message of type 'Percentage"
  (cl:format cl:nil "string Person~%int32 Percentage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Percentage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Person))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Percentage>))
  "Converts a ROS message object to a list"
  (cl:list 'Percentage
    (cl:cons ':Person (Person msg))
    (cl:cons ':Percentage (Percentage msg))
))
