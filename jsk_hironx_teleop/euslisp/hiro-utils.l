#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(hironxjsk-init)

(setq *left-coords-arrow* (arrow))
(setq *right-coords-arrow* (arrow))
(setq *left-target-arrow* (arrow))
(setq *right-target-arrow* (arrow))

(send *right-coords-arrow* :newcoords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
(send *left-coords-arrow* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
(send *right-target-arrow* :newcoords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
(send *left-target-arrow* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
(send (send *hironxjsk* :larm :end-coords) :assoc *left-coords-arrow*)
(send (send *hironxjsk* :rarm :end-coords) :assoc *right-coords-arrow*)
(setq object-list (list *hironxjsk* *right-coords-arrow* *left-coords-arrow* *left-target-arrow* *right-target-arrow*))
(objects object-list)

;; connection with robot and irt
(defun ir2ri (&optional (time 3000))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand))
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand))
  (send *ri* :wait-interpolation))
(defun ir2ri-hand ()
    (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand))
    (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand))
    (send *ri* :wait-interpolation))
(defun ri2ir ()
  (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector)))


;; ik
(defun solve-ik-larm (target-coords)
  (send *hironxjsk* :larm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))
(defun solve-ik-rarm (target-coords)
  (send *hironxjsk* :rarm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))


;; pose
(defun reset-pose ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 200) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 200) :world)
  (send *hironxjsk* :stop-grasp)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(600 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)
  (ir2ri 5000)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (setq left-reset-pose-coords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
  (setq right-reset-pose-coords (send *hironxjsk* :rarm :end-coords :copy-worldcoords))
  (setq reset-pose-angle-vector (send *hironxjsk* :angle-vector)))
(defun reset-pose-larm ()
  (send *left-coords-arrow* :move-to (send left-reset-pose-coords :copy-worldcoords) :world)
  (solve-ik-larm (send *left-coords-arrow* :copy-worldcoords))
  (ir2ri 5000))
(defun reset-pose-rarm ()
  (send *right-coords-arrow* :move-to (send right-reset-pose-coords :copy-worldcoords) :world)
  (solve-ik-rarm (send *right-coords-arrow* :copy-worldcoords))
  (ir2ri 5000))

(defun detect-pose()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(600 0 0)) :worldpos))
  (send *hironxjsk* :larm :move-end-pos #f(300 0 200) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(-300 0 200) :world)
  (ir2ri 5000)
  (send *hironxjsk* :stop-grasp)
  (ir2ri-hand))
(defun detect-pose-larm()
  (setq *left-target-arrow* (send (make-coords) :locate #f(325.704 482.265 165.629) :world))
  (solve-ik-larm *left-target-arrow*)
  (ir2ri))
(defun detect-pose-rarm()
  (setq *right-target-arrow* (send (make-coords) :locate #f(325.704 -482.265 165.629) :world))
  (solve-ik-rarm *right-target-arrow*)
  (ir2ri))
