#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://jsk_hironx_teleop/euslisp/hiro-utils.l")

;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "jsk_hironx_teleop")

(defun ros-vector3->float-vector (fvec)
  (float-vector (send fvec :x) (send fvec :y) (send fvec :z)))
(defun float-vector->ros-vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (/ (aref fvec 0) 1000.0)
            :y (/ (aref fvec 1) 1000.0)
            :z (/ (aref fvec 2) 1000.0)))

;; init
(ros::roseus "state-publisher")
(setq *left-target-arrow* (arrow))
(setq *right-target-arrow* (arrow))
(setq *left-end-coords* (arrow))
(setq *right-end-coords* (arrow))
(send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
(send *right-end-coords* :newcoords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
(send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
(send (Send *hironxjsk* :rarm :end-coords) :assoc *right-end-coords*)
(objects (list *hironxjsk* *left-target-arrow* *left-end-coords* *right-target-arrow* *right-end-coords*))

(ros::rate 50)
(ros::advertise "/hironx_imitation/state" jsk_hironx_teleop::FloatVector 1)
(setq state-msg (instance jsk_hironx_teleop::FloatVector :init))

(while t
  ;; hand
  (setq lhand-angle-vector (send *ri* :hand-angle-vector :lhand))
  (setq rhand-angle-vector (send *ri* :hand-angle-vector :rhand))
  (if (< (aref lhand-angle-vector 0) -10)
      (setq lhand-grasp t)
    (setq lhand-grasp nil))
  (if (< (aref rhand-angle-vector 0) 0)
      (setq rhand-grasp t)
    (setq rhand-grasp nil))
  (setq pub-time (ros::time-now))
  (send state-msg :header :stamp pub-time)
  (setq robot-state-joint-angle (coerce (mapcar #'deg2rad (coerce (send *ri* :state :potentio-vector) cons)) float-vector))
  (send state-msg :data (concatenate float-vector (subseq robot-state-joint-angle 0 6) (subseq robot-state-joint-angle 6 12) (if (eq rhand-grasp t) #f(1) #f(0)) (if (eq lhand-grasp t) #f(1) #f(0))))
  (ros::publish "/hironx_imitation/state" state-msg))
