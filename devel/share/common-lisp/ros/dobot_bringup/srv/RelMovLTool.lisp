; Auto-generated. Do not edit!


(cl:in-package dobot_bringup-srv)


;//! \htmlinclude RelMovLTool-request.msg.html

(cl:defclass <RelMovLTool-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:float
    :initform 0.0)
   (tool
    :reader tool
    :initarg :tool
    :type cl:integer
    :initform 0)
   (paramValue
    :reader paramValue
    :initarg :paramValue
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RelMovLTool-request (<RelMovLTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelMovLTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelMovLTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot_bringup-srv:<RelMovLTool-request> is deprecated: use dobot_bringup-srv:RelMovLTool-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:x-val is deprecated.  Use dobot_bringup-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:y-val is deprecated.  Use dobot_bringup-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:z-val is deprecated.  Use dobot_bringup-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:rx-val is deprecated.  Use dobot_bringup-srv:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:ry-val is deprecated.  Use dobot_bringup-srv:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:rz-val is deprecated.  Use dobot_bringup-srv:rz instead.")
  (rz m))

(cl:ensure-generic-function 'tool-val :lambda-list '(m))
(cl:defmethod tool-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:tool-val is deprecated.  Use dobot_bringup-srv:tool instead.")
  (tool m))

(cl:ensure-generic-function 'paramValue-val :lambda-list '(m))
(cl:defmethod paramValue-val ((m <RelMovLTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:paramValue-val is deprecated.  Use dobot_bringup-srv:paramValue instead.")
  (paramValue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelMovLTool-request>) ostream)
  "Serializes a message object of type '<RelMovLTool-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'tool)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'paramValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'paramValue))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelMovLTool-request>) istream)
  "Deserializes a message object of type '<RelMovLTool-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rz) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tool) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'paramValue) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'paramValue)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelMovLTool-request>)))
  "Returns string type for a service object of type '<RelMovLTool-request>"
  "dobot_bringup/RelMovLToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelMovLTool-request)))
  "Returns string type for a service object of type 'RelMovLTool-request"
  "dobot_bringup/RelMovLToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelMovLTool-request>)))
  "Returns md5sum for a message object of type '<RelMovLTool-request>"
  "cce5d7b80bc419b447b80cc47eb8fc50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelMovLTool-request)))
  "Returns md5sum for a message object of type 'RelMovLTool-request"
  "cce5d7b80bc419b447b80cc47eb8fc50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelMovLTool-request>)))
  "Returns full string definition for message of type '<RelMovLTool-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%float64 rx~%float64 ry~%float64 rz~%int32 tool~%string[] paramValue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelMovLTool-request)))
  "Returns full string definition for message of type 'RelMovLTool-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%float64 rx~%float64 ry~%float64 rz~%int32 tool~%string[] paramValue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelMovLTool-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'paramValue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelMovLTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RelMovLTool-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':tool (tool msg))
    (cl:cons ':paramValue (paramValue msg))
))
;//! \htmlinclude RelMovLTool-response.msg.html

(cl:defclass <RelMovLTool-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:integer
    :initform 0))
)

(cl:defclass RelMovLTool-response (<RelMovLTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelMovLTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelMovLTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot_bringup-srv:<RelMovLTool-response> is deprecated: use dobot_bringup-srv:RelMovLTool-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <RelMovLTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot_bringup-srv:res-val is deprecated.  Use dobot_bringup-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelMovLTool-response>) ostream)
  "Serializes a message object of type '<RelMovLTool-response>"
  (cl:let* ((signed (cl:slot-value msg 'res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelMovLTool-response>) istream)
  "Deserializes a message object of type '<RelMovLTool-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelMovLTool-response>)))
  "Returns string type for a service object of type '<RelMovLTool-response>"
  "dobot_bringup/RelMovLToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelMovLTool-response)))
  "Returns string type for a service object of type 'RelMovLTool-response"
  "dobot_bringup/RelMovLToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelMovLTool-response>)))
  "Returns md5sum for a message object of type '<RelMovLTool-response>"
  "cce5d7b80bc419b447b80cc47eb8fc50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelMovLTool-response)))
  "Returns md5sum for a message object of type 'RelMovLTool-response"
  "cce5d7b80bc419b447b80cc47eb8fc50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelMovLTool-response>)))
  "Returns full string definition for message of type '<RelMovLTool-response>"
  (cl:format cl:nil "~%int32 res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelMovLTool-response)))
  "Returns full string definition for message of type 'RelMovLTool-response"
  (cl:format cl:nil "~%int32 res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelMovLTool-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelMovLTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RelMovLTool-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RelMovLTool)))
  'RelMovLTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RelMovLTool)))
  'RelMovLTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelMovLTool)))
  "Returns string type for a service object of type '<RelMovLTool>"
  "dobot_bringup/RelMovLTool")