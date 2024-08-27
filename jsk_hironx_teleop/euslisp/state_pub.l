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

(defclass single-arm-handler
  :slots (arm-name *robot-state-pose* *robot-state-msg-pose* robot-state-msg-name-pose *robot-state-joint-angle* *robot-state-msg-joint-angle* robot-state-msg-name-joint-angle *arm-end-coords* *arm-tip-pos* *arm-end-pos* *arm-end-rpy* *grasp-button*))

(defmethod single-arm-handler
  (:init (&key ((:arm-name _arm-name) "larm"))
         (send self :set-val 'arm-name _arm-name)
         (send self :set-val '*robot-state-msg-pose* (instance jsk_hironx_teleop::FloatVector :init))
         (send self :set-val '*robot-state-msg-joint-angle* (instance jsk_hironx_teleop::FloatVector :init))
         (send self :set-val 'robot-state-msg-name-pose (format nil "/hironx_imitation/~A/robot_state/pose" arm-name))
         (send self :set-val 'robot-state-msg-name-joint-angle (format nil "/hironx_imitation/~A/robot_state/joint_angle" arm-name))
         (send self :setup-ros))

  (:setup-ros ()
              (ros::advertise (send self :get-val 'robot-state-msg-name-pose) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-state-msg-name-joint-angle) jsk_hironx_teleop::FloatVector 1))

  (:publish-state ()
                  (setq pub-time (ros::time-now))
                  (send self :set-val '*robot-state-pose*
                        (send *tfl* :lookup-transform "WAIST"
                              (format nil "~A_end_coords" (send self :get-val 'arm-name))
                              (ros::time 0)))
                  (send self :set-val '*robot-state-pose*
                        (concatenate float-vector
                                     (scale 0.01 (send (send self :get-val '*robot-state-pose*) :worldpos))
                                     (elt (rpy-angle (send (send self :get-val '*robot-state-pose*) :worldrot)) 0)
                                     #f(0)))
                 (send self :set-val '*robot-state-joint-angle* (coerce (mapcar #'deg2rad (coerce (send *ri* :state :potentio-vector) cons)) float-vector))
                 (send (send self :get-val '*robot-state-msg-pose*) :data (send self :get-val '*robot-state-pose*))
                 (send (send self :get-val '*robot-state-msg-joint-angle*) :data (send self :get-val '*robot-state-joint-angle*))
                 (send (send self :get-val '*robot-state-msg-pose*) :header :stamp pub-time)
                 (send (send self :get-val '*robot-state-msg-joint-angle*) :header :stamp pub-time)
                 (ros::publish (send self :get-val 'robot-state-msg-name-pose) (send self :get-val '*robot-state-msg-pose*))
                 (ros::publish (send self :get-val 'robot-state-msg-name-joint-angle) (send self :get-val '*robot-state-msg-joint-angle*))))

(defclass dual-arm-handler
  :slots (larm-handler))

(defmethod dual-arm-handler
  (:init ()
   (ros::roseus "state-publisher")
   (setq *left-target-arrow* (arrow))
   (setq *left-end-coords* (arrow))
   (send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
   (objects (list *hironxjsk* *left-target-arrow* *left-end-coords*))
   (send self :set-val 'larm-handler (instance single-arm-handler :init :arm-name "larm")))

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :publish-state))

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (send self :loop-call))))


(defun main (&rest args)
  (setq node (instance dual-arm-handler :init))
  (send node :run))

(main)
