(load "package://jsk_pcl_ros/euslisp/bounding-box-to-cube.l")
(load "package://jsk_hironx_teleop/euslisp/topic-utils.l")

(defun valid-boxes (msg)
  (let (box)
    (when (> (length (send msg :boxes)) 0)
      (dotimes (i (length (send msg :boxes)))
        (setq box (elt (send msg :boxes) i))
        ;; (when (and (> (send box :dimensions :y) 0.1)
        ;;            (> (send box :dimensions :z) 0.05)
        ;;            (> (send box :dimensions :x) 0.2))
        (when (and (> (send box :dimensions :y) 0.12)
                   (> (send box :dimensions :z) 0.05)
                   (> (send box :dimensions :x) 0.17))
          (return-from valid-boxes box))))
    nil))

(defun valid-paper (msg)
  (let (box)
    (when (> (length (send msg :boxes)) 0)
      (dotimes (i (length (send msg :boxes)))
        (setq box (elt (send msg :boxes) i))
        ;; (when (and (> (send box :dimensions :y) 0.01)
        ;;            (> (send box :dimensions :z) 0.01))
        (when (and (> (send box :dimensions :y) 0.02)
                   (> (send box :dimensions :z) 0.06))
          (return-from valid-paper box))))
    nil))

(defun find-bbox (topic-name &key (condition) (timeout 10))
  (let* (msg
         ros-bbox
         tareget-bounding-box
         camera-to-box-transform
         base-to-box-transform)

    (while (or (null msg) (= (length (send msg :boxes)) 0))
      (setq msg (one-shot-subscribe topic-name
                                    jsk_recognition_msgs::BoundingBoxArray
                                    :timeout (* timeout 1000)
                                    :condition condition))
      )

    (setq ros-bbox (valid-boxes msg))
    (setq tareget-bounding-box (bounding-box->cube ros-bbox))

    (setq base-to-camera-transform (send *tfl* :lookup-transform
                                         "WAIST" (send msg :header :frame_id)
                                         (ros::time 0)))
    (setq base-to-box-transform
          (send
           (send base-to-camera-transform :copy-worldcoords)
           :transform tareget-bounding-box))
    (send tareget-bounding-box :newcoords (send base-to-box-transform :copy-worldcoords))
    tareget-bounding-box))

(defun find-paper (topic-name &key (condition) (timeout 10))
  (let* (msg
         ros-bbox
         tareget-bounding-box
         camera-to-box-transform
         base-to-box-transform)

    (while (or (null msg) (= (length (send msg :boxes)) 0))
      (setq msg (one-shot-subscribe topic-name
                                    jsk_recognition_msgs::BoundingBoxArray
                                    :timeout (* timeout 1000)
                                    :condition condition))
      )

    (setq ros-bbox (valid-paper msg))
    (setq tareget-bounding-box (bounding-box->cube ros-bbox))

    (setq base-to-camera-transform (send *tfl* :lookup-transform
                                         "WAIST" (send msg :header :frame_id)
                                         (ros::time 0)))
    (setq base-to-box-transform
          (send
           (send base-to-camera-transform :copy-worldcoords)
           :transform tareget-bounding-box))
    (send tareget-bounding-box :newcoords (send base-to-box-transform :copy-worldcoords))
    tareget-bounding-box))
