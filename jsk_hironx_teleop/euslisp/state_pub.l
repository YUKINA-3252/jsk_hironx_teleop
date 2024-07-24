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
  :slots (arm-name *robot-state-msg* robot-state-msg-name *arm-end-coords* *arm-tip-pos* *arm-end-pos* *arm-end-rpy* *robot-state*))

(defmethod single-arm-handler
  (:init (&key ((:arm-name _arm-name) "larm"))
         (send self :set-val 'arm-name _arm-name)
         (send self :set-val '*robot-state-msg* (instance jsk_hironx_teleop::FloatVector :init))
         (send self :set-val 'robot-state-msg-name (format nil "/hironx_imitation/~A/robot_state" arm-name))
         (send self :setup-ros))

  (:setup-ros ()
              (ros::advertise (send self :get-val 'robot-state-msg-name) jsk_hironx_teleop::FloatVector 1))

  (:publish-state ()
                  (if (string= (send self :get-val 'arm-name) "larm")
                      (progn
                        (send self :set-val '*arm-end-coords* (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                        (send self :set-val '*arm-tip-pos* (send (send *hironxjsk* :get :lhand-tip-coords) :worldpos)))
                    (progn
                      (send self :set-val '*arm-end-coords* (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
                      (send self :set-val '*arm-tip-pos* (send (send *hironxjsk* :get :rhand-tip-coords) :worldpos))))
                  (send self :set-val '*arm-end-pos* (send *arm-end-coords* :worldpos))
                  (send self :set-val '*arm-end-rpy*
                        (coerce (elt (rpy-angle (send *arm-end-coords* :worldrot)) 0) float-vector))
                  (send self :set-val '*robot-state*
                        (concatenate float-vector
                                     (send self :get-val '*arm-end-pos*)
                                     (send self :get-val '*arm-end-rpy*)
                                     (send self :get-val '*arm-tip-pos*)))
                  (send (send self :get-val '*robot-state-msg*) :data (send self :get-val '*robot-state*))
                  (send (send self :get-val '*robot-state-msg*) :header :stamp (ros::time-now))
                  (ros::publish (send self :get-val 'robot-state-msg-name) (send self :get-val '*robot-state-msg*))))

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
