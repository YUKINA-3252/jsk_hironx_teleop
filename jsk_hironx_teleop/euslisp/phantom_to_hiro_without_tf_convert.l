#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://jsk_hironx_teleop/euslisp/hiro-utils.l")

;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "omni_msgs")
(ros::roseus-add-msgs "jsk_hironx_teleop")

(setq force-sensor-pub nil)
(setq force-sensor-topic-name "/force_sensor_status")

(setq theta (deg2rad 10))

(defun ros-vector3->float-vector (fvec)
  (float-vector (send fvec :x) (send fvec :y) (send fvec :z)))
;; (defun float-vector->ros-vector3 (fvec)
;;   (instance geometry_msgs::Vector3 :init
;;             :x (/ (aref fvec 0) 1000.0)
;;             :y (/ (aref fvec 1) 1000.0)
;;             :z (/ (aref fvec 2) 1000.0)))
(defun float-vector->ros-vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (aref fvec 0)
            :y (aref fvec 1)
            :z (aref fvec 2)))

(defun convert-coords (msg)
  (let (b c)
    (setq b (ros::tf-pose->coords (send msg :pose)))
    (setq c (make-coords :rot (copy-object (send b :rot))))
    (send c :rotate pi/2 :x :world)
    (send c :locate (send b :worldpos) :world)
    c))

(setq state-msg (instance jsk_hironx_teleop::FloatVector :init))
(setq action-msg (instance jsk_hironx_teleop::FloatVector :init))
(ros::advertise "/hironx_imitation/state" jsk_hironx_teleop::FloatVector 1)
(ros::advertise "/hironx_imitation/action" jsk_hironx_teleop::FloatVector 1)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale side target-pose target-force zero-force force-topic force-pub-topic *robot-state-pose* *robot-state-msg-pose* robot-state-msg-name-pose *robot-state-joint-angle* *robot-state-msg-joint-angle* robot-state-msg-name-joint-angle *robot-action-pose* *robot-action-msg-pose* robot-action-msg-name-pose *robot-action-joint-angle* *robot-action-msg-joint-angle* robot-action-msg-name-joint-angle *enable-button* *grasp-button* *touch-trans* *touch-rot* delta-pos *left-target-arrow* *right-target-arrow* cur-device-pose pub-time dev-topic initial-target-pose time-a time-b device-coords-for-robot c initial-hand-coords initial-pen-coords initial-convert-pen-coords pen-coords float-vector-target-force initial-float-vector-target-force cur-float-vector-target-force rotation-matrix-from-endcoords-to-device))

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
   ;; force
   (send self :set-val 'force-topic (format nil "/off_~Ahsensor" (if (string= (send self :get-val 'arm-name) "larm") "l" "r")))
   ;; (send self :set-val 'force-topic (format nil "/~A_contact_sensor_force_moment_topic" (send self :get-val 'arm-name)))
   (send self :set-val 'force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
   ;; grasp
   (send self :set-val '*grasp-button* _*grasp-button*)
   (send self :set-val '*enable-button* "initial")
   (send self :set-val 'time-a (ros::time-now))
   ;; state
   (send self :set-val '*robot-state-msg-pose* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val '*robot-state-msg-joint-angle* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val 'robot-state-msg-name-pose (format nil "/hironx_imitation/~A/robot_state/pose" arm-name))
   (send self :set-val 'robot-state-msg-name-joint-angle (format nil "/hironx_imitation/~A/robot_state/joint_angle" arm-name))
   ;; action
   (send self :set-val '*robot-action-msg-pose* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val '*robot-action-msg-joint-angle* (instance jsk_hironx_teleop::FloatVector :init))
   (send self :set-val 'robot-action-msg-name-pose (format nil "/hironx_imitation/~A/robot_action/pose" arm-name))
   (send self :set-val 'robot-action-msg-name-joint-angle (format nil "/hironx_imitation/~A/robot_action/joint_angle" arm-name))
   (send self :set-val '*left-target-arrow* (arrow))
   (send self :set-val '*right-target-arrow* (arrow))
   (send self :setup-ros))

  (:setup-ros ()
              (send self :set-val 'dev-topic (format nil "/~A/phantom/state" (send self :get-val 'master-dev-name)))
              ;; force
              (ros::subscribe (send self :get-val 'force-topic) geometry_msgs::WrenchStamped #'send self :force-cb 1)
              (ros::advertise (send self :get-val 'force-pub-topic) omni_msgs::OmniFeedback 1)
              ;; state
              (ros::advertise (send self :get-val 'robot-state-msg-name-pose) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-state-msg-name-joint-angle) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-action-msg-name-pose) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-action-msg-name-joint-angle) jsk_hironx_teleop::FloatVector 1)
              (ros::subscribe (send self :get-val 'dev-topic) omni_msgs::OmniState #'send self :device-state-sub 1)
              (send self :set-val 'target-pose (send self :set-initial-pose))
              ;; force sensor state
              (ros::subscribe force-sensor-topic-name std_msgs::String #'send self :force-sensor-state-check 1)
              )

  (:device-state-sub (msg)
                     (send self :set-val '*touch-trans*
                           (float-vector (send msg :pose :position :x)
                                         (send msg :pose :position :y)
                                         (send msg :pose :position :z)))
                     (send self :set-val '*touch-rot*
                           (float-vector (send msg :pose :orientation :w)
                                         (send msg :pose :orientation :x)
                                         (send msg :pose :orientation :y)
                                         (send msg :pose :orientation :z)))
                     (send self :set-val 'cur-device-pose (convert-coords msg))
                     ;; initial setting
                     (if (not (send self :get-val 'device-coords-for-robot))
                         (progn
                           (send self :set-val 'device-coords-for-robot (make-coords))
                           (send (send self :get-val 'device-coords-for-robot) :rotate (deg2rad -90) :z :local)))
                     (if (not (send self :get-val 'initial-pen-coords))
                         (send self :set-val 'initial-pen-coords (send (make-coords) :locate (send self :get-val '*touch-trans*) :world)))
                     (if (not (send self :get-val 'initial-convert-pen-coords))
                         (send self :set-val 'initial-convert-pen-coords (send (send self :get-val 'cur-device-pose) :copy-worldcoords)))
                     (if (not (send self :get-val 'initial-hand-coords))
                         (if (string= (send self :get-val 'arm-name) "larm")
                             (progn
                               (print "initial")
                               (send self :set-val 'initial-hand-coords (send *hironxjsk* :larm :end-coords :copy-worldcoords)))
                           (send self :set-val 'initial-hand-coords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))))
                     ;; match the value of white button and msg :locked
                     (if (string= (send self :get-val '*enable-button*) "initial")
                         (cond ((send msg :locked) (send self :set-val '*enable-button* nil))
                               ((not (send msg :locked)) (send self :set-val '*enable-button* t))))
                     ;; gripper's states
                     (if (and (not (send self :get-val '*grasp-button*)) (send msg :close_gripper))
                         (progn
                           (send self :set-val '*grasp-button* t)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (close-lhand-rope)
                             (close-rhand-rope))
                           (ir2ri-hand)))
                     (if (and (send self :get-val '*grasp-button*) (not (send msg :close_gripper)))
                         (progn
                           (send self :set-val '*grasp-button* nil)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (open-lhand-rope)
                             (open-rhand-rope))
                           (ir2ri-hand)))
                     ;; Process when changing from unlocked to locked
                     (if (and (send self :get-val '*enable-button*) (send msg :locked))
                         (send self :set-val '*enable-button* nil))
                     ;; Process when changing from locked to unlocked
                     (if (and (not (send self :get-val '*enable-button*)) (not (send msg :locked)))
                         (progn
                           (send self :set-val 'initial-pen-coords (send (make-coords) :locate (send self :get-val '*touch-trans*) :world))
                           (send self :set-val 'cur-device-pose (convert-coords msg))
                           (send self :set-val 'initial-convert-pen-coords (send (send self :get-val 'cur-device-pose) :copy-worldcoords))
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (send self :set-val 'initial-hand-coords (send *hironxjsk* :larm :end-coords :copy-worldcoords))

                             (send self :set-val 'initial-hand-coords (send *hironxjsk* :rarm :end-coords :copy-worldcoords)))
                           (send self :set-val '*enable-button* t)))
                     ;; when locked
                     (if (not (send self :get-val '*enable-button*))
                         (send self :set-val 'delta-pos (float-vector 0 0 0)))
                     ;; when unlocked
                     (if (send self :get-val '*enable-button*)
                         (progn
                           ;; delta pos
                           (send self :set-val 'delta-pos
                                 (float-vector
                                  (- (- (elt (send self :get-val '*touch-trans*) 1) (elt (send (send self :get-val 'initial-pen-coords) :worldpos) 1)))
                                  (- (elt (send self :get-val '*touch-trans*) 0) (elt (send (send self :get-val 'initial-pen-coords) :worldpos) 0))
                                  (- (elt (send self :get-val '*touch-trans*) 2) (elt (send (send self :get-val 'initial-pen-coords) :worldpos) 2))))
                           ;; rotate
                           (send self :set-val 'pen-coords (send (send self :get-val 'cur-device-pose) :copy-worldcoords))
                           (send self :set-val 'c (send (send (send self :get-val 'initial-convert-pen-coords) :copy-worldcoords) :transformation (send (send self :get-val 'pen-coords) :copy-worldcoords) (send (send self :get-val 'device-coords-for-robot) :copy-worldcoords)))
                           (send (send self :get-val 'target-pose) :move-to (send (send (send self :get-val 'initial-hand-coords) :copy-worldcoords) :transform (send (send self :get-val 'c) :copy-worldcoords) :world) :world)
                           (send (send self :get-val 'target-pose) :locate
                                 (send (send (send (send self :get-val 'initial-hand-coords) :copy-worldcoords)
                                             :translate
                                             (scale (send self :get-val 'pos-scale)
                                                    (send self :get-val 'delta-pos)) :world) :pos) :world)
                           ))
                     ;; solve ik
                     (if (string= (send self :get-val 'arm-name) "larm")
                         (progn
                           (send (send self :get-val '*left-target-arrow*) :move-to (send (send self :get-val 'target-pose) :copy-worldcoords) :world)
                           (solve-ik-larm (send (send self :get-val '*left-target-arrow*) :copy-worldcoords))
                           (send self :set-val '*robot-action-pose*
                                 (concatenate float-vector
                                              (scale 0.01 (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :worldpos))
                                              (elt (rpy-angle (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :worldrot)) 0)
                                              (if (send self :get-val '*grasp-button*) #f(1) #f(0))))
                           (print "test"))
                       (progn
                         (send (send self :get-val '*right-target-arrow*) :move-to (send (send self :get-val 'target-pose) :copy-worldcoords) :world)
                         (solve-ik-rarm (send (send self :get-val '*right-target-arrow*) :copy-worldcoords))
                         (send self :set-val '*robot-action-pose*
                               (concatenate float-vector
                                            (scale 0.01 (send (send *hironxjsk* :rarm :end-coords :copy-worldcoords) :worldpos))
                                            (elt (rpy-angle (send (send *hironxjsk* :rarm :end-coords :copy-worldcoords) :worldrot)) 0)
                                            (if (send self :get-val '*grasp-button*) #f(1) #f(0))))))
                     ;; publish action
                     (send self :set-val 'pub-time (ros::time-now))
                     (send self :set-val '*robot-action-joint-angle* (concatenate float-vector (coerce (mapcar #'deg2rad (coerce (send *hironxjsk* :angle-vector) cons)) float-vector) (if (send self :get-val '*grasp-button*) #f(1) #f(0))))
                     (send (send self :get-val '*robot-action-msg-pose*) :data (send self :get-val '*robot-action-pose*))
                     (send (send self :get-val '*robot-action-msg-joint-angle*) :data (send self :get-val '*robot-action-joint-angle*))
                     (send (send self :get-val '*robot-action-msg-pose*) :header :stamp (send self :get-val 'pub-time))
                     (send (send self :get-val '*robot-action-msg-joint-angle*) :header :stamp (send self :get-val 'pub-time))
                     ;; (ros::publish (send self :get-val 'robot-action-msg-name-pose) (send self :get-val '*robot-action-msg-pose*))
                     ;; (ros::publish (send self :get-val 'robot-action-msg-name-joint-angle) (send self :get-val '*robot-action-msg-joint-angle*))
                     )

  (:force-cb (msg)
             (send self :set-val 'rotation-matrix-from-endcoords-to-device
                   (if (string= (send self :get-val 'arm-name) "larm")
                       (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :rot)
                     (send (send *hironxjsk* :rarm :end-coords :copy-worldcoords) :rot)))
             (if (eq (send self :get-val 'initial-float-vector-target-force) nil)
                 (send self :set-val 'initial-float-vector-target-force
                       (m* (send self :get-val 'rotation-matrix-from-endcoords-to-device)
                           (make-matrix 3 1 (list (list (send msg :wrench :force :x))
                                                  (list (send msg :wrench :force :y))
                                                  (list (send msg :wrench :force :z)))))))
             (send self :set-val 'cur-float-vector-target-force
                   (m* (send self :get-val 'rotation-matrix-from-endcoords-to-device)
                       (make-matrix 3 1 (list (list (send msg :wrench :force :x))
                                              (list (send msg :wrench :force :y))
                                              (list (send msg :wrench :force :z))))))
             (send self :set-val 'float-vector-target-force #f(0 0 0))
             (if (and (eq force-sensor-pub t) (> (abs (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 0) (matrix-row (send self :get-val 'initial-float-vector-target-force) 0)) 0)) 2.0))
                 (setf (elt (send self :get-val 'float-vector-target-force) 0)
                       (* (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 0) (matrix-row (send self :get-val 'initial-float-vector-target-force) 0)) 0) (send self :get-val 'force-scale))))
             (if (and (eq force-sensor-pub t) (> (abs (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 1) (matrix-row (send self :get-val 'initial-float-vector-target-force) 1)) 0)) 2.0))
                 (setf (elt (send self :get-val 'float-vector-target-force) 1)
                       (* (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 1) (matrix-row (send self :get-val 'initial-float-vector-target-force) 1)) 0) (send self :get-val 'force-scale))))
             (if (and (eq force-sensor-pub t) (> (abs (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 2) (matrix-row (send self :get-val 'initial-float-vector-target-force) 2)) 0)) 2.0))
                 (setf (elt (send self :get-val 'float-vector-target-force) 2)
                       (* (elt (v- (matrix-row (send self :get-val 'cur-float-vector-target-force) 2) (matrix-row (send self :get-val 'initial-float-vector-target-force) 2)) 0) (send self :get-val 'force-scale))))
             (send (send self :get-val 'target-force) :force (float-vector->ros-vector3 (send self :get-val 'float-vector-target-force)))
             )

  (:apply-target-force ()
                       (ros::publish (send self :get-val 'force-pub-topic) (send self :get-val 'target-force)))

  (:force-sensor-state-check (msg)
                             (if (equal (send msg :data) "force sensor on")
                                 (setq force-sensor-pub t)))

  (:update-robot ()
                 (send self :set-val 'pub-time (ros::time-now))
                 (send self :set-val '*robot-state-pose*
                       (send *tfl* :lookup-transform "WAIST"
                             (format nil "~A_end_coords" (send self :get-val 'arm-name))
                             (ros::time 0)))
                 (send self :set-val '*robot-state-pose*
                       (concatenate float-vector
                                    (scale 0.01 (send (send self :get-val '*robot-state-pose*) :worldpos))
                                    (elt (rpy-angle (send (send self :get-val '*robot-state-pose*) :worldrot)) 0)
                                    (if (send self :get-val '*grasp-button*) #f(1) #f(0))))
                 (send self :set-val '*robot-state-joint-angle* (concatenate float-vector (coerce (mapcar #'deg2rad (coerce (send *ri* :state :potentio-vector) cons)) float-vector) (if (send self :get-val '*grasp-button*) #f(1) #f(0))))
                 (send (send self :get-val '*robot-state-msg-pose*) :data (send self :get-val '*robot-state-pose*))
                 (send (send self :get-val '*robot-state-msg-joint-angle*) :data (send self :get-val '*robot-state-joint-angle*))
                 (send (send self :get-val '*robot-state-msg-pose*) :header :stamp pub-time)
                 (send (send self :get-val '*robot-state-msg-joint-angle*) :header :stamp pub-time)
                 ;; (ros::publish (send self :get-val 'robot-state-msg-name-pose) (send self :get-val '*robot-state-msg-pose*))
                 ;; (ros::publish (send self :get-val 'robot-state-msg-name-joint-angle) (send self :get-val '*robot-state-msg-joint-angle*))
                 )

  (:set-initial-pose ()
                     (if (string= (send self :get-val 'arm-name) "larm")
                         (progn
                           (send self :set-val 'initial-target-pose (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                           )
                       (progn
                         (send self :set-val 'initial-target-pose (send *hironxjsk* :rarm :end-coords :copy-worldcoords)))))

  (:run ()
        (ros::spin-once)))


(defclass dual-arm-handler
  :slots (pos-scale-dual force-scale-dual *imitation-mode* larm-handler rarm-handler left-grasp-state right-grasp-state))

(defmethod dual-arm-handler
  (:init
   (&key ((:pos-scale-dual _pos-scale-dual) 1.0) ((:force-scale-dual _force-scale-dual) 0.1) ((:*imitation-mode* _*imitation-mode*) t))
   (ros::roseus "dual-phantom-master")
   (setq *left-end-coords* (arrow))
   (send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
   (setq *right-end-coords* (arrow))
   (send *right-end-coords* :newcoords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :rarm :end-coords) :assoc *right-end-coords*)
   ;; (objects (list *hironxjsk* *left-target-arrow-dual* *left-end-coords*))
   (send self :set-val 'left-grasp-state nil)
   (send self :set-val 'right-grasp-state nil)
   (send self :set-val 'pos-scale-dual _pos-scale-dual)
   (send self :set-val 'force-scale-dual _force-scale-dual)
   (send self :set-val '*imitation-mode* _*imitation-mode*)
   (send self :set-val 'larm-handler (instance single-arm-handler :init :arm-name "larm" :master-dev-name "left_device" :pos-scale (send self :get-val 'pos-scale-dual) :force-scale (send self :get-val 'force-scale-dual)))
   (send self :set-val 'rarm-handler (instance single-arm-handler :init :arm-name "rarm" :master-dev-name "right_device" :pos-scale (send self :get-val 'pos-scale-dual) :force-scale (send self :get-val 'force-scale-dual)))
   (if (send self :get-val '*imitation-mode*)
       (ros::subscribe "/hironx_imitation/action" jsk_hironx_teleop::FloatVector #'send self :imitation 1))
   (send (send self :get-val 'larm-handler) :run)
   (send (send self :get-val 'rarm-handler) :run)
   )

  (:imitation (msg)
              (print "imitation")
              (setq angle-vector (concatenate float-vector (coerce (mapcar #'rad2deg (coerce (concatenate float-vector (subseq (send msg :data) 0 6) (subseq (send msg :data) 6 12)) cons)) float-vector) (subseq (send *ri* :state :potentio-vector) 12 15)))
              (send *hironxjsk* :angle-vector angle-vector)
              ;; grasp
              (if (and (> (elt (send msg :data) 12) 0.8) (eq (send self :get-val 'right-grasp-state) nil))
                  (progn
                    (close-rhand-rope-imitation)
                    (send self :set-val 'right-grasp-state t)))
              (if (and (< (elt (send msg :data) 12) 0.8) (eq (send self :get-val 'right-grasp-state) t))
                  (progn
                    (open-rhand-rope-imitation)
                    (send self :set-val 'right-grasp-state nil)))
              (if (and (> (elt (send msg :data) 13) 0.8) (eq (send self :get-val 'left-grasp-state) nil))
                  (progn
                    (close-lhand-rope-imitation)
                    (send self :set-val 'left-grasp-state t)))
              (if (and (< (elt (send msg :data) 13) 0.8) (eq (send self :get-val 'left-grasp-state) t))
                  (progn
                    (open-lhand-rope-imitation)
                    (send self :set-val 'left-grasp-state nil)))
              )

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :run)
              (send (send self :get-val 'rarm-handler) :run)
              (if (not (send self :get-val '*imitation-mode*))
                  (progn
                    ;; publish state and action topic
                    (if (and (not (eq (send (send self :get-val 'rarm-handler) :get-val '*robot-state-joint-angle*) nil)) (not (eq (send (send self :get-val 'larm-handler) :get-val '*robot-state-joint-angle*) nil)) (not (eq (send (send self :get-val 'rarm-handler) :get-val '*robot-action-joint-angle*) nil)) (not (eq (send (send self :get-val 'larm-handler) :get-val '*robot-action-joint-angle*) nil)))
                        (progn
                          (send state-msg :header :stamp (send (send self :get-val 'larm-handler) :get-val 'pub-time))
                          (send action-msg :header :stamp (send (send self :get-val 'larm-handler) :get-val 'pub-time))
                          (send state-msg :data (concatenate float-vector (subseq (send (send self :get-val 'rarm-handler) :get-val '*robot-state-joint-angle*) 0 6) (subseq (send (send self :get-val 'larm-handler) :get-val '*robot-state-joint-angle*) 6 12) (float-vector (elt (send (send self :get-val 'rarm-handler) :get-val '*robot-state-joint-angle*) 15)) (float-vector (elt (send (send self :get-val 'larm-handler) :get-val '*robot-state-joint-angle*) 15))))
                          (ros::publish "/hironx_imitation/state" state-msg)
                          (send action-msg :data (concatenate float-vector (subseq (send (send self :get-val 'rarm-handler) :get-val '*robot-action-joint-angle*) 0 6) (subseq (send (send self :get-val 'larm-handler) :get-val '*robot-action-joint-angle*) 6 12) (float-vector (elt (send (send self :get-val 'rarm-handler) :get-val '*robot-action-joint-angle*) 15)) (float-vector (elt (send (send self :get-val 'larm-handler) :get-val '*robot-action-joint-angle*) 15))))
                          (ros::publish "/hironx_imitation/action" action-msg)))))
              ;; (send *ri* :angle-vector (send *hironxjsk* :angle-vector) (* 1.5 (/ 1000 50)))
              (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
              (send *irtviewer* :draw-objects)
              ;; force
              (send (send self :get-val 'larm-handler) :apply-target-force)
              (send (send self :get-val 'rarm-handler) :apply-target-force)
              ;; update state and action
              (send (send self :get-val 'larm-handler) :update-robot)
              (send (send self :get-val 'rarm-handler) :update-robot)
              )

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (send self :loop-call))))

(defun main (&rest args)
  (setq node (instance dual-arm-handler :init :pos-scale-dual 1.0 :force-scale-dual 5.0 :*imitation-mode* nil))
  (send node :run))

(main)
