;; Auto-generated. Do not edit!


(when (boundp 'action_tutorial::DoDishesResult)
  (if (not (find-package "ACTION_TUTORIAL"))
    (make-package "ACTION_TUTORIAL"))
  (shadow 'DoDishesResult (find-package "ACTION_TUTORIAL")))
(unless (find-package "ACTION_TUTORIAL::DODISHESRESULT")
  (make-package "ACTION_TUTORIAL::DODISHESRESULT"))

(in-package "ROS")
;;//! \htmlinclude DoDishesResult.msg.html


(defclass action_tutorial::DoDishesResult
  :super ros::object
  :slots (_total_dishes_cleaned ))

(defmethod action_tutorial::DoDishesResult
  (:init
   (&key
    ((:total_dishes_cleaned __total_dishes_cleaned) 0)
    )
   (send-super :init)
   (setq _total_dishes_cleaned (round __total_dishes_cleaned))
   self)
  (:total_dishes_cleaned
   (&optional __total_dishes_cleaned)
   (if __total_dishes_cleaned (setq _total_dishes_cleaned __total_dishes_cleaned)) _total_dishes_cleaned)
  (:serialization-length
   ()
   (+
    ;; uint32 _total_dishes_cleaned
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _total_dishes_cleaned
       (write-long _total_dishes_cleaned s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _total_dishes_cleaned
     (setq _total_dishes_cleaned (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get action_tutorial::DoDishesResult :md5sum-) "58faa44dda59c6545bea63ff57491cbe")
(setf (get action_tutorial::DoDishesResult :datatype-) "action_tutorial/DoDishesResult")
(setf (get action_tutorial::DoDishesResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# result
uint32 total_dishes_cleaned

")



(provide :action_tutorial/DoDishesResult "58faa44dda59c6545bea63ff57491cbe")

