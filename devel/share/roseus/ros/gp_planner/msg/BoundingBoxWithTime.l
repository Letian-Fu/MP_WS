;; Auto-generated. Do not edit!


(when (boundp 'gp_planner::BoundingBoxWithTime)
  (if (not (find-package "GP_PLANNER"))
    (make-package "GP_PLANNER"))
  (shadow 'BoundingBoxWithTime (find-package "GP_PLANNER")))
(unless (find-package "GP_PLANNER::BOUNDINGBOXWITHTIME")
  (make-package "GP_PLANNER::BOUNDINGBOXWITHTIME"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxWithTime.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gp_planner::BoundingBoxWithTime
  :super ros::object
  :slots (_header _left_x _right_x _up_y _down_y ))

(defmethod gp_planner::BoundingBoxWithTime
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_x __left_x) 0.0)
    ((:right_x __right_x) 0.0)
    ((:up_y __up_y) 0.0)
    ((:down_y __down_y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_x (float __left_x))
   (setq _right_x (float __right_x))
   (setq _up_y (float __up_y))
   (setq _down_y (float __down_y))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_x
   (&optional __left_x)
   (if __left_x (setq _left_x __left_x)) _left_x)
  (:right_x
   (&optional __right_x)
   (if __right_x (setq _right_x __right_x)) _right_x)
  (:up_y
   (&optional __up_y)
   (if __up_y (setq _up_y __up_y)) _up_y)
  (:down_y
   (&optional __down_y)
   (if __down_y (setq _down_y __down_y)) _down_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _left_x
    4
    ;; float32 _right_x
    4
    ;; float32 _up_y
    4
    ;; float32 _down_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _left_x
       (sys::poke _left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_x
       (sys::poke _right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _up_y
       (sys::poke _up_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _down_y
       (sys::poke _down_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _left_x
     (setq _left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_x
     (setq _right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _up_y
     (setq _up_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _down_y
     (setq _down_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get gp_planner::BoundingBoxWithTime :md5sum-) "3f7bcaf4671d6882f7146f67396528cb")
(setf (get gp_planner::BoundingBoxWithTime :datatype-) "gp_planner/BoundingBoxWithTime")
(setf (get gp_planner::BoundingBoxWithTime :definition-)
      "# 文件名：BoundingBoxWithTime.msg
std_msgs/Header header
float32 left_x   # 左边界的x坐标
float32 right_x  # 右边界的x坐标
float32 up_y     # 上边界的y坐标
float32 down_y   # 下边界的y坐标

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :gp_planner/BoundingBoxWithTime "3f7bcaf4671d6882f7146f67396528cb")


