#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://jsk_hironx_teleop/euslisp/hiro-utils.l")

;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "omni_msgs")

(defun ros-vector3->float-vector (fvec)
  (float-vector (send fvec :x) (send fvec :y) (send fvec :z)))
(defun float-vector->ros-vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (/ (aref fvec 0) 1000.0)
            :y (/ (aref fvec 1) 1000.0)
            :z (/ (aref fvec 2) 1000.0)))

(defvar *tf-listener* nil)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale send-rot side master-initial-q hironx-initial-q q-m-p *tf-listener* current-frame-id moving target-pose target-force device-target-pos zero-force force-topic force-pub-topic gripper-status))

(defmethod single-arm-handler
  (:init
   (&key ((:robot-id _robot-id) "dual_hironx") ((:arm-name _arm-name) "larm") ((:master-dev-name _master-dev-name) "left_device") ((:pos-scale _pos-scale) 1.0) ((:force-scale _force-scale) 0.1) ((:send-rot _send-rot) t) ((:side _side) "left") ((:gripper-status _gripper-status) nil))
   (send self :set-val 'robot-id _robot-id)
   (send self :set-val 'arm-name _arm-name)
   (send self :set-val 'master-dev-name _master-dev-name)
   (send self :set-val 'pos-scale _pos-scale)
   (send self :set-val 'force-scale _force-scale)
   (send self :set-val 'target-force (instance omni_msgs::OmniFeedback :init))
   (send self :set-val 'zero-force nil)
   (send self :set-val 'master-initial-q (quaternion2matrix #f(0.5 -0.5 -0.5 -0.5)))
   (send self :set-val 'hironx-initial-q (quaternion2matrix #f(0 1.0 0 0)))
   (send self :set-val 'q-m-p (m* (send self :get-val 'hironx-initial-q) (inverse-matrix (send self :get-val 'master-initial-q))))
   (send self :set-val 'current-frame-id (format nil "~A_end_coords" (send self :get-val 'arm-name)))
   (send self :set-val 'moving nil)
   (send self :set-val '*tf-listener* (instance ros::transform-listener :init))
   (send self :set-val 'force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
   (send self :set-val 'gripper-status _gripper-status)
   (send self :setup-ros))

  (:setup-ros ()
              (send *tf-listener* :wait-for-transform (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) (send self :get-val 'current-frame-id) (ros::time-now) 3.6)
              (setq dev-topic (format nil "/~A/phantom/state" (send self :get-val 'master-dev-name)))
              (send self :set-val 'force-topic (format nil "/~Ahsensor" (if (eq (send self :get-val 'arm-name) "larm") "l" "r")))
              ;; (setq force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
              (ros::subscribe dev-topic omni_msgs::OmniState #'send self :device-state-sub 2000)
              (ros::subscribe force-topic geometry_msgs::WrenchStamped #'send self :force-cb 1000)
              (ros::advertise (send self :get-val 'force-pub-topic) omni_msgs::OmniFeedback 1)
              (send self :set-val 'target-pose (send self :set-initial-pose)))

  (:device-state-sub (msg)
                     ;; not move target pose when locked
                     (if (send msg :locked)
                         (progn
                           (setq moving nil)
                           (break)))
                     ;; ;; gripper's status
                     ;; (if (not (and (send self :get-val 'gripper-status) (send msg :close_gripper)))
                     ;;     (progn
                     ;;       (if (send self :get-val 'gripper-status)
                     ;;           (progn
                     ;;             (if (eq (send self :get-val 'arm-name) "larm")
                     ;;                 (send *ri* :stop-grasp :larm)
                     ;;               (send *ri* :stop-grasp :rarm))
                     ;;             ;; (ir2ri-hand)
                     ;;             (send self :set-val 'gripper-status nil))
                     ;;         (progn
                     ;;           (if (eq (send self :get-val 'arm-name) "larm")
                     ;;               (send *ri* :start-grasp :larm)
                     ;;             (send *ri* :start-grasp :rarm))
                     ;;           ;; (ir2ri-hand)
                     ;;           (send self :set-val 'gripper-status t)))))
                     ;; initial device-target-pos
                     (if (not (send self :get-val 'device-target-pos))
                           (send self :set-val 'device-target-pos (send msg :pose :position)))
                     ;; update target-pose
                     (setq moving t)
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 0)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 0) (* (- (- (send msg :pose :position :y) (send (send self :get-val 'device-target-pos) :y))) (send self :get-val 'pos-scale))))
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 1)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 1) (* (- (send msg :pose :position :x) (send (send self :get-val 'device-target-pos) :x)) (send self :get-val 'pos-scale))))
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 2)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 2) (* (- (send msg :pose :position :z) (send (send self :get-val 'device-target-pos) :z)) (send self :get-val 'pos-scale))))
                     (send self :set-val 'device-target-pos (send msg :pose :position))
                     (if send-rot
                         (ros::ros-info "test")
                         (progn
                           (setq cur-q (quaternion2matrix (float-vector
                                                           (send msg :pose :orientation :w)
                                                           (send msg :pose :orientation :x)
                                                           (send msg :pose :orientation :y)
                                                           (send msg :pose :orientation :z))))
                           (setq tar-q (m* (send self :get-val 'q-m-p) cur-q))
                           (setq adjust-tar-q (quaternion2matrix (float-vector
                                                                  (- (elt (matrix2quaternion tar-q) 0))
                                                                  (elt (matrix2quaternion tar-q) 1)
                                                                  (- (elt (matrix2quaternion tar-q) 2))
                                                                  (elt (matrix2quaternion tar-q) 3))))
                           (send (send self :get-val 'target-pose) :Euler
                                 (elt (elt (euler-angle adjust-tar-q) 0) 0)
                                 (elt (elt (euler-angle adjust-tar-q) 0) 1)
                                 (elt (elt (euler-angle adjust-tar-q) 0) 2)))))

  (:force-cb (msg)
             (if (eq (send self :get-val 'zero-force) nil)
                 (send self :set-val 'zero-force (float-vector (send msg :wrench :force :x)
                                                               (send msg :wrench :force :y)
                                                               (send msg :wrench :force :z))))
             (setq float-vector-target-force (ros-vector3->float-vector (send (send self :get-val 'target-force) :force)))
             (setf (elt float-vector-target-force 0) (* (- (send msg :wrench :force :y) (elt (send self :get-val 'zero-force) 0)) (send self :get-val 'force-scale)))
             (setf (elt float-vector-target-force 1) (* (- (- (send msg :wrench :force :x) (elt (send self :get-val 'zero-force) 1))) (send self :get-val 'force-scale)))
             (setf (elt float-vector-target-force 2) (* (- (send msg :wrench :force :z) (elt (send self :get-val 'zero-force) 2)) (send self :get-val 'force-scale)))
             (send (send self :get-val 'target-force) :force (float-vector->ros-vector3 float-vector-target-force)))

  (:apply-target-force ()
                       (ros::publish (send self :get-val 'force-pub-topic) (send self :get-val 'target-force)))

  (:set-initial-pose ()
                     (setq current-frame (make-coords))
                     (setq joint0-link-to-end-effector-transform (send *tfl* :lookup-transform (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) current-frame-id (ros::time 0)))
                     (send current-frame :move-to (send joint0-link-to-end-effector-transform :copy-worldcoords)))

  (:transform-waist-based ()
                          (setq base-to-joint0-link (send *tfl* :lookup-transform
                                                          "WAIST" (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) (ros::time 0)))
                          (setq base-to-target-transform (send (send base-to-joint0-link :copy-worldcoords) :transform (send (send self :get-val 'target-pose) :copy-worldcoords))))

  (:run ()
        (ros::spin-once)))

(defclass dual-arm-handler
  :slots (pos-scale-dual force-scale-dual larm-handler device-to-endcoords-transform))

(defmethod dual-arm-handler
  (:init
   (&key ((:pos-scale-dual _pos-scale-dual) 1.0) ((:force-scale-dual _force-scale-dual) 0.1))
   (ros::roseus "dual-phantom-master")
   (setq *left-target-arrow* (arrow))
   (setq *left-end-coords* (arrow))
   (send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
   (objects (list *hironxjsk* *left-target-arrow* *left-end-coords*))
   (send self :set-val 'pos-scale-dual _pos-scale-dual)
   (send self :set-val 'force-scale-dual _force-scale-dual)
   (send self :set-val 'larm-handler (instance single-arm-handler :init :arm-name "larm" :master-dev-name "left_device" :pos-scale (send self :get-val 'pos-scale-dual) :force-scale (send self :get-val 'force-scale-dual)))
   (send (send self :get-val 'larm-handler) :run)
   (send *left-target-arrow* :move-to (send (send (send self :get-val 'larm-handler) :transform-waist-based) :copy-worldcoords) :world)
   (send self :set-val 'device-to-endcoords-transform (send (send *left-target-arrow* :copy-worldcoords) :transformation (send *left-end-coords* :copy-worldcoords))))

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :run)
              (send *left-target-arrow* :move-to (send (send (send (send self :get-val 'larm-handler) :transform-waist-based) :copy-worldcoords) :transform (send (send self :get-val 'device-to-endcoords-transform) :copy-worldcoords)) :world)
              (solve-ik-larm (send *left-target-arrow* :copy-worldcoords))
              (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
              ;; (send *ri* :wait-interpolation)
              (send *irtviewer* :draw-objects)
              ;; force
              (send (send self :get-val 'larm-handler) :apply-target-force))

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 10)
   (while t
     (send self :loop-call)
     (ros::sleep))))


(defun main (&rest args)
  (setq node (instance dual-arm-handler :init :pos-scale-dual 0.5 :force-scale-dual 100.0))
  (send node :run))

(main)
