; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Transcript.msg.html

(cl:defclass <Transcript> (roslisp-msg-protocol:ros-message)
  ((Timestamp
    :reader Timestamp
    :initarg :Timestamp
    :type cl:real
    :initform 0)
   (Person
    :reader Person
    :initarg :Person
    :type cl:string
    :initform "")
   (Speech
    :reader Speech
    :initarg :Speech
    :type cl:string
    :initform ""))
)

(cl:defclass Transcript (<Transcript>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Transcript>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Transcript)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Transcript> is deprecated: use beginner_tutorials-msg:Transcript instead.")))

(cl:ensure-generic-function 'Timestamp-val :lambda-list '(m))
(cl:defmethod Timestamp-val ((m <Transcript>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Timestamp-val is deprecated.  Use beginner_tutorials-msg:Timestamp instead.")
  (Timestamp m))

(cl:ensure-generic-function 'Person-val :lambda-list '(m))
(cl:defmethod Person-val ((m <Transcript>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Person-val is deprecated.  Use beginner_tutorials-msg:Person instead.")
  (Person m))

(cl:ensure-generic-function 'Speech-val :lambda-list '(m))
(cl:defmethod Speech-val ((m <Transcript>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:Speech-val is deprecated.  Use beginner_tutorials-msg:Speech instead.")
  (Speech m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Transcript>) ostream)
  "Serializes a message object of type '<Transcript>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'Timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'Timestamp) (cl:floor (cl:slot-value msg 'Timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Person))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Person))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Speech))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Speech))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Transcript>) istream)
  "Deserializes a message object of type '<Transcript>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Person) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Person) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Speech) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Speech) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Transcript>)))
  "Returns string type for a message object of type '<Transcript>"
  "beginner_tutorials/Transcript")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Transcript)))
  "Returns string type for a message object of type 'Transcript"
  "beginner_tutorials/Transcript")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Transcript>)))
  "Returns md5sum for a message object of type '<Transcript>"
  "ae4f77e2a32a76d57c67744b255ce34b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Transcript)))
  "Returns md5sum for a message object of type 'Transcript"
  "ae4f77e2a32a76d57c67744b255ce34b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Transcript>)))
  "Returns full string definition for message of type '<Transcript>"
  (cl:format cl:nil "time Timestamp~%string Person~%string Speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Transcript)))
  "Returns full string definition for message of type 'Transcript"
  (cl:format cl:nil "time Timestamp~%string Person~%string Speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Transcript>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'Person))
     4 (cl:length (cl:slot-value msg 'Speech))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Transcript>))
  "Converts a ROS message object to a list"
  (cl:list 'Transcript
    (cl:cons ':Timestamp (Timestamp msg))
    (cl:cons ':Person (Person msg))
    (cl:cons ':Speech (Speech msg))
))
