#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://jsk_hironx_teleop/euslisp/hiro-utils.l")

;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "omni_msgs")
(ros::roseus-add-msgs "jsk_hironx_teleop")

(setq time-a nil)
(setq time-b nil)
(setq interval-time nil)
(setq collection-start nil)

(defun ros-vector3->float-vector (fvec)
  (float-vector (send fvec :x) (send fvec :y) (send fvec :z)))
(defun float-vector->ros-vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (/ (aref fvec 0) 1000.0)
            :y (/ (aref fvec 1) 1000.0)
            :z (/ (aref fvec 2) 1000.0)))

(defvar *tf-listener* nil)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale side target-pose target-force device-target-pos zero-force force-topic force-pub-topic transformation-matrix *arm-end-coords* *arm-end-pos* *arm-end-rpy* *arm-tip-pos* *robot-state* *robot-state-msg* robot-state-msg-name *robot-action* *robot-action-msg* robot-action-msg-name *enable-button* *grasp-button* *touch-trans* *touch-rot* data-collect-trigger-topic-name initial-device-pose base-to-endcoords-transform))

(defmethod single-arm-handler
  (:init
   (&key ((:robot-id _robot-id) "dual_hironx") ((:arm-name _arm-name) "larm") ((:master-dev-name _master-dev-name) "left_device") ((:pos-scale _pos-scale) 1.0) ((:force-scale _force-scale) 0.1) ((:side _side) "left") ((:*grasp-button* _*grasp-button*) nil))
   (send self :set-val 'robot-id _robot-id)
   (send self :set-val 'arm-name _arm-name)
   (send self :set-val 'master-dev-name _master-dev-name)
   (send self :set-val 'pos-scale _pos-scale)
   (send self :set-val 'force-scale _force-scale)
   (send self :set-val 'target-force (instance omni_msgs::OmniFeedback :init))
   (send self :set-val 'zero-force nil)
   (send self :set-val 'transformation-matrix (make-matrix 3 3 (list (list 0 -1 0) (list 1 0 0) (list 0 0 1))))
   (send self :set-val 'force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
   (send self :set-val '*grasp-button* _*grasp-button*)
   (send self :set-val '*enable-button* "initial")
   (setq time-a (ros::time-now))
   (send self :set-val '*robot-state-msg* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val 'robot-state-msg-name (format nil "/hironx_imitation/~A/robot_state" arm-name))
   (send self :set-val '*robot-action-msg* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val 'robot-action-msg-name (format nil "/hironx_imitation/~A/robot_action" arm-name))
   (send self :set-val 'data-collect-trigger-topic-name (format nil "/hironx_imitation/~A/data_collector/record" arm-name))
   (send self :setup-ros))

  (:setup-ros ()
              (setq dev-topic (format nil "/~A/phantom/state" (send self :get-val 'master-dev-name)))
              (send self :set-val 'force-topic (format nil "/~Ahsensor" (if (string= (send self :get-val 'arm-name) "larm") "l" "r")))
              ;; (setq force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
              (ros::subscribe dev-topic omni_msgs::OmniState #'send self :device-state-sub 1)
              (ros::subscribe force-topic geometry_msgs::WrenchStamped #'send self :force-cb 1)
              (ros::advertise (send self :get-val 'force-pub-topic) omni_msgs::OmniFeedback 1)
              (ros::advertise (send self :get-val 'robot-state-msg-name) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-action-msg-name) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'data-collect-trigger-topic-name) std_msgs::String 1)
              (send self :set-val 'target-pose (send self :set-initial-pose)))

  (:device-state-sub (msg)
                     (send self :set-val '*touch-trans*
                           (float-vector (send msg :pose :position :y)
                                         (send msg :pose :position :x)
                                         (send msg :pose :position :z)))
                     (send self :set-val '*touch-rot*
                           (float-vector (send msg :pose :orientation :w)
                                         (send msg :pose :orientation :x)
                                         (send msg :pose :orientation :y)
                                         (send msg :pose :orientation :z)))
                     ;; initial device-target-pos
                     (if (not (send self :get-val 'device-target-pos))
                         (send self :set-val 'device-target-pos (send msg :pose :position)))
                     ;; match the value of white button and msg :locked
                     (if (string= (send self :get-val '*enable-button*) "initial")
                         (cond ((send msg :locked) (send self :set-val '*enable-button* nil))
                               ((not (send msg :locked)) (send self :set-val '*enable-button* t))))
                     (if (not (send self :get-val 'initial-device-pose))
                         (send self :set-initial-device-pose))
                     ;; gripper's states
                     (if (and (not *grasp-button*) (send msg :close_gripper))
                         (progn
                           (send self :set-val '*grasp-button* t)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (send *hironxjsk* :start-grasp :larm)
                             (send *hironxjsk* :start-grasp :rarm))
                           (ir2ri-hand)))
                     (if (and (send self :get-val '*grasp-button*) (not (send msg :close_gripper)))
                         (progn
                           (send self :set-val '*grasp-button* nil)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (send *hironxjsk* :stop-grasp :larm)
                             (send *hironxjsk* :stop-grasp :rarm))
                           (ir2ri-hand)))
                     ;; Process when changing from unlocked to locked
                     (if (and (send self :get-val '*enable-button*) (send msg :locked))
                         (progn
                           (send self :set-val '*enable-button* nil)
                           (send self :cal-interval-time)
                           (cond ((and (not collection-start) (< interval-time 1))
                                  (progn
                                    (send self :data-collection-start)
                                    (setq collection-start t)))
                                 ((and collection-start (< interval-time 1))
                                  (send self :data-collection-stop)))))
                     ;; Process when changing from locked to unlocked
                     (if (and (not (send self :get-val '*enable-button*)) (not (send msg :locked)))
                         (progn
                           (send self :cal-interval-time)
                           (send self :set-val 'device-target-pos (send msg :pose :position))
                           (send self :set-initial-device-pose)
                           (send self :set-val '*enable-button* t)
                           (cond ((and (not collection-start) (< interval-time 1))
                                  (progn
                                    (send self :data-collection-start)
                                    (setq collection-start t)))
                                 ((and collection-start (< interval-time 1))
                                  (send self :data-collection-stop)))))
                     ;; when unlocked
                     (if (send self :get-val '*enable-button*)
                         (progn
                           ;; update pos
                           (send (send self :get-val 'target-pose) :translate
                                 (float-vector (* (- (elt (send self :get-val '*touch-trans*) 0) (send (send self :get-val 'device-target-pos) :y)) (send self :get-val 'pos-scale))
                                               (* (- (- (elt (send self :get-val '*touch-trans*) 1) (send (send self :get-val 'device-target-pos) :x))) (send self :get-val 'pos-scale))
                                               (* (- (elt (send self :get-val '*touch-trans*) 2) (send (send self :get-val 'device-target-pos) :z)) (send self :get-val 'pos-scale))) :local)
                           (send self :set-val 'device-target-pos (send msg :pose :position))
                           ;; update rot
                           (setq cur-device-rot (quaternion2matrix (send self :get-val '*touch-rot*)))
                           (setq cur-device-pose
                                 (send (make-coords) :Euler
                                       (elt (elt (euler-angle cur-device-rot) 0) 0)
                                       (elt (elt (euler-angle cur-device-rot) 0) 1)
                                       (elt (elt (euler-angle cur-device-rot) 0) 2)))
                           (send cur-device-pose :translate (send self :get-val '*touch-trans*) :world)
                           (setq device-transform (send (send self :get-val 'initial-device-pose) :transformation (send cur-device-pose :copy-worldcoords)))
                           (setq device-transform-rot (send device-transform :rot))
                           (setq endcoords-transform-rot (m* (m* (inverse-matrix (send self :get-val 'transformation-matrix)) (m* device-transform-rot (send self :get-val 'transformation-matrix))) (send self :get-val 'base-to-endcoords-transform)))
                           (send (send self :get-val 'target-pose) :Euler
                                 (elt (elt (euler-angle endcoords-transform-rot) 0) 0)
                                 (elt (elt (euler-angle endcoords-transform-rot) 0) 1)
                                 (elt (elt (euler-angle endcoords-transform-rot) 0) 2))))
                     (send self :set-val '*robot-action*
                           (concatenate float-vector
                                        (send self :get-val '*touch-trans*)
                                        (send self :get-val '*touch-rot*)
                                        (float-vector (if (send self :get-val '*grasp-button*) 1 0))))
                     (send (send self :get-val '*robot-action-msg*) :data (send self :get-val '*robot-action*))
                     (send (send self :get-val '*robot-action-msg*) :header :stamp (ros::time-now))
                     (ros::publish (send self :get-val 'robot-action-msg-name) (send self :get-val '*robot-action-msg*)))

  (:set-initial-device-pose ()
                                       (setq initial-device-rotation-matrix (quaternion2matrix (send self :get-val '*touch-rot*)))
                                       (send self :set-val 'initial-device-pose
                                             (send (make-coords) :Euler
                                                   (elt (elt (euler-angle initial-device-rotation-matrix) 0) 0)
                                                   (elt (elt (euler-angle initial-device-rotation-matrix) 0) 1)
                                                   (elt (elt (euler-angle initial-device-rotation-matrix) 0) 2)))
                                       (send (send self :get-val 'initial-device-pose) :translate (send self :get-val '*touch-trans*) :world)
                                       (send self :set-val 'base-to-endcoords-transform (send (send self :get-val 'target-pose) :worldrot)))

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

  (:update-robot ()
                 (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector))
                 (if (string= (send self :get-val 'arm-name) "larm")
                     (progn
                       (send self :set-val '*arm-end-coords* (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                       (send self :set-val '*arm-tip-pos* (send (send *hironxjsk* :get :lhand-tip-coords) :worldpos)))
                   (progn
                     (send self :set-val '*arm-end-coords* (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
                     (send self :set-val '*arm-tip-pos* (send (send *hironxjsk* :get :rhand-tip-coords) :worldpos))))
                 (send self :set-val '*arm-end-pos* (send *arm-end-coords* :worldpos))
                 (send self :set-val '*arm-end-rpy* (coerce (elt
                                              (rpy-angle (send *arm-end-coords* :worldrot)) 0) float-vector))
                 (send self :set-val '*robot-state*
                       (concatenate float-vector
                                    (send self :get-val '*arm-end-pos*)
                                    (send self :get-val '*arm-end-rpy*)
                                    (send self :get-val '*arm-tip-pos*)))
                 (send (send self :get-val '*robot-state-msg*) :data (send self :get-val '*robot-state*))
                 (send (send self :get-val '*robot-state-msg*) :header :stamp (ros::time-now))
                 (ros::publish (send self :get-val 'robot-state-msg-name) (send self :get-val '*robot-state-msg*)))

  (:set-initial-pose ()
                     (if (string= (send self :get-val 'arm-name) "larm")
                         (setq initial-target-pose (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                       (setq initial-target-pose (send *hironxjsk* :rarm :end-coords :copy-worldcoords))))

  (:cal-interval-time ()
                      (setq time-b (ros::time-now))
                      (setq interval-time (- (send time-b :sec) (send time-a :sec)))
                      (setq time-a time-b))

  (:data-collection-start ()
                          (ros::publish (send self :get-val 'data-collect-trigger-topic-name) (instance std_msgs::String :init :data "start")))

  (:data-collection-stop ()
                        (ros::publish (send self :get-val 'data-collect-trigger-topic-name) (instance std_msgs::String :init :data "stop")))

  (:run ()
        (ros::spin-once)))

(defclass dual-arm-handler
  :slots (pos-scale-dual force-scale-dual larm-handler))

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
   (send *left-target-arrow* :move-to (send *hironxjsk* :larm :end-coords :copy-worldcoords) :world))

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :run)
              (send *left-target-arrow* :move-to (send (send (send self :get-val 'larm-handler) :get-val 'target-pose) :copy-worldcoords) :world)
              (solve-ik-larm (send *left-target-arrow* :copy-worldcoords))
              ;; (send *ri* :angle-vector (send *hironxjsk* :angle-vector) (* 1.5 (/ 1000 50)))
              ;; (send *ri* :wait-interpolation)
              (send *irtviewer* :draw-objects)
              ;; force
              ;; (send (send self :get-val 'larm-handler) :apply-target-force)
              ;; imitation
              (send (send self :get-val 'larm-handler) :update-robot))

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (send self :loop-call))))


(defun main (&rest args)
  (setq node (instance dual-arm-handler :init :pos-scale-dual 1.0 :force-scale-dual 50.0))
  (send node :run))

(main)
a
