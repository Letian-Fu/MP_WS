; Auto-generated. Do not edit!


(cl:in-package gp_planner-msg)


;//! \htmlinclude BoundingBoxWithTime.msg.html

(cl:defclass <BoundingBoxWithTime> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_x
    :reader left_x
    :initarg :left_x
    :type cl:float
    :initform 0.0)
   (right_x
    :reader right_x
    :initarg :right_x
    :type cl:float
    :initform 0.0)
   (up_y
    :reader up_y
    :initarg :up_y
    :type cl:float
    :initform 0.0)
   (down_y
    :reader down_y
    :initarg :down_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass BoundingBoxWithTime (<BoundingBoxWithTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBoxWithTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBoxWithTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gp_planner-msg:<BoundingBoxWithTime> is deprecated: use gp_planner-msg:BoundingBoxWithTime instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BoundingBoxWithTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gp_planner-msg:header-val is deprecated.  Use gp_planner-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_x-val :lambda-list '(m))
(cl:defmethod left_x-val ((m <BoundingBoxWithTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gp_planner-msg:left_x-val is deprecated.  Use gp_planner-msg:left_x instead.")
  (left_x m))

(cl:ensure-generic-function 'right_x-val :lambda-list '(m))
(cl:defmethod right_x-val ((m <BoundingBoxWithTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gp_planner-msg:right_x-val is deprecated.  Use gp_planner-msg:right_x instead.")
  (right_x m))

(cl:ensure-generic-function 'up_y-val :lambda-list '(m))
(cl:defmethod up_y-val ((m <BoundingBoxWithTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gp_planner-msg:up_y-val is deprecated.  Use gp_planner-msg:up_y instead.")
  (up_y m))

(cl:ensure-generic-function 'down_y-val :lambda-list '(m))
(cl:defmethod down_y-val ((m <BoundingBoxWithTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gp_planner-msg:down_y-val is deprecated.  Use gp_planner-msg:down_y instead.")
  (down_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBoxWithTime>) ostream)
  "Serializes a message object of type '<BoundingBoxWithTime>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'up_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'down_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBoxWithTime>) istream)
  "Deserializes a message object of type '<BoundingBoxWithTime>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'up_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'down_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBoxWithTime>)))
  "Returns string type for a message object of type '<BoundingBoxWithTime>"
  "gp_planner/BoundingBoxWithTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBoxWithTime)))
  "Returns string type for a message object of type 'BoundingBoxWithTime"
  "gp_planner/BoundingBoxWithTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBoxWithTime>)))
  "Returns md5sum for a message object of type '<BoundingBoxWithTime>"
  "3f7bcaf4671d6882f7146f67396528cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBoxWithTime)))
  "Returns md5sum for a message object of type 'BoundingBoxWithTime"
  "3f7bcaf4671d6882f7146f67396528cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBoxWithTime>)))
  "Returns full string definition for message of type '<BoundingBoxWithTime>"
  (cl:format cl:nil "# 文件名：BoundingBoxWithTime.msg~%std_msgs/Header header~%float32 left_x   # 左边界的x坐标~%float32 right_x  # 右边界的x坐标~%float32 up_y     # 上边界的y坐标~%float32 down_y   # 下边界的y坐标~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBoxWithTime)))
  "Returns full string definition for message of type 'BoundingBoxWithTime"
  (cl:format cl:nil "# 文件名：BoundingBoxWithTime.msg~%std_msgs/Header header~%float32 left_x   # 左边界的x坐标~%float32 right_x  # 右边界的x坐标~%float32 up_y     # 上边界的y坐标~%float32 down_y   # 下边界的y坐标~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBoxWithTime>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBoxWithTime>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBoxWithTime
    (cl:cons ':header (header msg))
    (cl:cons ':left_x (left_x msg))
    (cl:cons ':right_x (right_x msg))
    (cl:cons ':up_y (up_y msg))
    (cl:cons ':down_y (down_y msg))
))
