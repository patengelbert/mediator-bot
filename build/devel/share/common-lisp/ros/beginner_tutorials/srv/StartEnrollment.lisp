; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-srv)


;//! \htmlinclude StartEnrollment-request.msg.html

(cl:defclass <StartEnrollment-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass StartEnrollment-request (<StartEnrollment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartEnrollment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartEnrollment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<StartEnrollment-request> is deprecated: use beginner_tutorials-srv:StartEnrollment-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <StartEnrollment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:name-val is deprecated.  Use beginner_tutorials-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartEnrollment-request>) ostream)
  "Serializes a message object of type '<StartEnrollment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartEnrollment-request>) istream)
  "Deserializes a message object of type '<StartEnrollment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartEnrollment-request>)))
  "Returns string type for a service object of type '<StartEnrollment-request>"
  "beginner_tutorials/StartEnrollmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartEnrollment-request)))
  "Returns string type for a service object of type 'StartEnrollment-request"
  "beginner_tutorials/StartEnrollmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartEnrollment-request>)))
  "Returns md5sum for a message object of type '<StartEnrollment-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartEnrollment-request)))
  "Returns md5sum for a message object of type 'StartEnrollment-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartEnrollment-request>)))
  "Returns full string definition for message of type '<StartEnrollment-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartEnrollment-request)))
  "Returns full string definition for message of type 'StartEnrollment-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartEnrollment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartEnrollment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartEnrollment-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude StartEnrollment-response.msg.html

(cl:defclass <StartEnrollment-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartEnrollment-response (<StartEnrollment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartEnrollment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartEnrollment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<StartEnrollment-response> is deprecated: use beginner_tutorials-srv:StartEnrollment-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StartEnrollment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:success-val is deprecated.  Use beginner_tutorials-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartEnrollment-response>) ostream)
  "Serializes a message object of type '<StartEnrollment-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartEnrollment-response>) istream)
  "Deserializes a message object of type '<StartEnrollment-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartEnrollment-response>)))
  "Returns string type for a service object of type '<StartEnrollment-response>"
  "beginner_tutorials/StartEnrollmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartEnrollment-response)))
  "Returns string type for a service object of type 'StartEnrollment-response"
  "beginner_tutorials/StartEnrollmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartEnrollment-response>)))
  "Returns md5sum for a message object of type '<StartEnrollment-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartEnrollment-response)))
  "Returns md5sum for a message object of type 'StartEnrollment-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartEnrollment-response>)))
  "Returns full string definition for message of type '<StartEnrollment-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartEnrollment-response)))
  "Returns full string definition for message of type 'StartEnrollment-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartEnrollment-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartEnrollment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartEnrollment-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartEnrollment)))
  'StartEnrollment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartEnrollment)))
  'StartEnrollment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartEnrollment)))
  "Returns string type for a service object of type '<StartEnrollment>"
  "beginner_tutorials/StartEnrollment")