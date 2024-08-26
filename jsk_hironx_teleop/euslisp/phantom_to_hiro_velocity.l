(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(setq *left-target-arrow* (arrow))
(send *left-target-arrow* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))

(hironxjsk-init)
(objects (list *hironxjsk* *left-target-arrow*))

(defun ir2ri (&optional (time 3000))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand))
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand))
  (send *ri* :wait-interpolation))
(defun ri2ir ()
  (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector)))

(defun reset-pose ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 200) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 200) :world)
  (send *hironxjsk* :stop-grasp)
  (ir2ri 5000)
  (setq left-initial-coords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
  (setq right-initial-coords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
  (setq initial-angle-vector (send *hironxjsk* :angle-vector)))

(defun solve-ik-larm (target-coords)
  (send *hironxjsk* :larm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))
(defun solve-ik-rarm (target-coords)
  (send *hironxjsk* :rarm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis nil))

(while t
  (setq larm-joint0-link-to-target-pose (send (one-shot-subscribe "/phantom_to_hironx/larm_target_pose" geometry_msgs::PoseStamped :timeout 1000) :pose))
  (setq larm-joint0-link-to-target-coords (ros::tf-pose->coords larm-joint0-link-to-target-pose))
  (setq base-to-larm-joint0-link (send *tfl* :lookup-transform
                                       "WAIST" "LARM_JOINT0_Link"
                                       (ros::time 0)))
  (setq base-to-target-transform (send (send base-to-larm-joint0-link :copy-worldcoords) :transform (send larm-joint0-link-to-target-coords :copy-worldcoords)))
  (send *left-target-arrow* :move-to (send base-to-target-transform :copy-worldcoords) :world)
  (send *irtviewer* :draw-objects))
)
