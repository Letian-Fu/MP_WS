; Auto-generated. Do not edit!


(cl:in-package dobot_v4_bringup-srv)


;//! \htmlinclude SetHoldRegs-request.msg.html

(cl:defclass <SetHoldRegs-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (addr
    :reader addr
    :initarg :addr
    :type cl:integer
    :initform 0)
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0)
   (valTab
    :reader valTab
    :initarg :valTab
    :type cl:string
    :initform "")
   (valType
    :reader valType
    :initarg :valType
    :type cl:string
    :initform ""))
)

(cl:defclass SetHoldRegs-request (<SetHoldRegs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHoldRegs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHoldRegs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot_v4_bringup-srv:<SetHoldRegs-request> is deprecated: use dobot_v4_bringup-srv:SetHoldRegs-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <SetHoldRegs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:index-val is deprecated.  Use dobot_v4_bringup-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'addr-val :lambda-list '(m))
(cl:defmethod addr-val ((m <SetHoldRegs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:addr-val is deprecated.  Use dobot_v4_bringup-srv:addr instead.")
  (addr m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <SetHoldRegs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:count-val is deprecated.  Use dobot_v4_bringup-srv:count instead.")
  (count m))

(cl:ensure-generic-function 'valTab-val :lambda-list '(m))
(cl:defmethod valTab-val ((m <SetHoldRegs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:valTab-val is deprecated.  Use dobot_v4_bringup-srv:valTab instead.")
  (valTab m))

(cl:ensure-generic-function 'valType-val :lambda-list '(m))
(cl:defmethod valType-val ((m <SetHoldRegs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:valType-val is deprecated.  Use dobot_v4_bringup-srv:valType instead.")
  (valType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHoldRegs-request>) ostream)
  "Serializes a message object of type '<SetHoldRegs-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'addr)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'valTab))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'valTab))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'valType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'valType))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHoldRegs-request>) istream)
  "Deserializes a message object of type '<SetHoldRegs-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'addr) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'valTab) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'valTab) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'valType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'valType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHoldRegs-request>)))
  "Returns string type for a service object of type '<SetHoldRegs-request>"
  "dobot_v4_bringup/SetHoldRegsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHoldRegs-request)))
  "Returns string type for a service object of type 'SetHoldRegs-request"
  "dobot_v4_bringup/SetHoldRegsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHoldRegs-request>)))
  "Returns md5sum for a message object of type '<SetHoldRegs-request>"
  "e0d42a09df85575ba7301debdafa6447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHoldRegs-request)))
  "Returns md5sum for a message object of type 'SetHoldRegs-request"
  "e0d42a09df85575ba7301debdafa6447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHoldRegs-request>)))
  "Returns full string definition for message of type '<SetHoldRegs-request>"
  (cl:format cl:nil "int32    index~%int32    addr~%int32    count~%string   valTab~%string   valType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHoldRegs-request)))
  "Returns full string definition for message of type 'SetHoldRegs-request"
  (cl:format cl:nil "int32    index~%int32    addr~%int32    count~%string   valTab~%string   valType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHoldRegs-request>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'valTab))
     4 (cl:length (cl:slot-value msg 'valType))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHoldRegs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHoldRegs-request
    (cl:cons ':index (index msg))
    (cl:cons ':addr (addr msg))
    (cl:cons ':count (count msg))
    (cl:cons ':valTab (valTab msg))
    (cl:cons ':valType (valType msg))
))
;//! \htmlinclude SetHoldRegs-response.msg.html

(cl:defclass <SetHoldRegs-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:integer
    :initform 0))
)

(cl:defclass SetHoldRegs-response (<SetHoldRegs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHoldRegs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHoldRegs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot_v4_bringup-srv:<SetHoldRegs-response> is deprecated: use dobot_v4_bringup-srv:SetHoldRegs-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <SetHoldRegs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_v4_bringup-srv:res-val is deprecated.  Use dobot_v4_bringup-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHoldRegs-response>) ostream)
  "Serializes a message object of type '<SetHoldRegs-response>"
  (cl:let* ((signed (cl:slot-value msg 'res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHoldRegs-response>) istream)
  "Deserializes a message object of type '<SetHoldRegs-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHoldRegs-response>)))
  "Returns string type for a service object of type '<SetHoldRegs-response>"
  "dobot_v4_bringup/SetHoldRegsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHoldRegs-response)))
  "Returns string type for a service object of type 'SetHoldRegs-response"
  "dobot_v4_bringup/SetHoldRegsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHoldRegs-response>)))
  "Returns md5sum for a message object of type '<SetHoldRegs-response>"
  "e0d42a09df85575ba7301debdafa6447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHoldRegs-response)))
  "Returns md5sum for a message object of type 'SetHoldRegs-response"
  "e0d42a09df85575ba7301debdafa6447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHoldRegs-response>)))
  "Returns full string definition for message of type '<SetHoldRegs-response>"
  (cl:format cl:nil "int32 res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHoldRegs-response)))
  "Returns full string definition for message of type 'SetHoldRegs-response"
  (cl:format cl:nil "int32 res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHoldRegs-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHoldRegs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHoldRegs-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHoldRegs)))
  'SetHoldRegs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHoldRegs)))
  'SetHoldRegs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHoldRegs)))
  "Returns string type for a service object of type '<SetHoldRegs>"
  "dobot_v4_bringup/SetHoldRegs")