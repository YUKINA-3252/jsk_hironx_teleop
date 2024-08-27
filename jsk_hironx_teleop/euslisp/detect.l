(load "models/arrow-object.l")
(load "package://jsk_hironx_teleop/euslisp/box-utils.l")

(defmethod coordinates
  (:x-axis () (matrix-column rot 0))
  (:y-axis () (matrix-column rot 1))
  (:z-axis () (matrix-column rot 2)))

(defun scale-vector (vector factor)
  (mapcar #'(lambda (x) (* x factor)) vector))

(defun f2list (flist)
  (list (elt flist 0) (elt flist 1) (elt flist 2)))

(defun list2f (slist)
  (float-vector (elt slist 0) (elt slist 1) (elt slist 2)))

(defun detect-box()
  (let (vertical)
    (setq *bbox-topic-name* "/tabletop_cpi_decomposer_head/boxes")
    (setq *target-bbox* (find-bbox *bbox-topic-name* :condition #'valid-boxes))
    (setq *target-bbox-arrow* (arrow))
    (send *target-bbox-arrow* :newcoords (send *target-bbox* :copy-worldcoords))
    (setq surface-location (list
                         (send (send *target-bbox* :copy-worldcoords) :translate (list2f (scale-vector (f2list (send *target-bbox-arrow* :x-axis)) (/ (x-of-cube *target-bbox*) 2))) :world)
                         (send (send (send *target-bbox* :copy-worldcoords) :translate (list2f (scale-vector (f2list (send *target-bbox-arrow* :x-axis)) (/ (x-of-cube *target-bbox*) -2))) :world) :rotate pi :z)
                         (send (send *target-bbox* :copy-worldcoords) :translate (list2f (scale-vector (f2list (send *target-bbox-arrow* :y-axis)) (/ (y-of-cube *target-bbox*) 2))) :world)
                         (send (send (send *target-bbox* :copy-worldcoords) :translate (list2f (scale-vector (f2list (send *target-bbox-arrow* :y-axis)) (/ (y-of-cube *target-bbox*) -2))) :world) :rotate pi :z)))
    (setq surface-normal-vector (list
                              (send *target-bbox-arrow* :x-axis) (list2f (scale-vector (f2list (send *target-bbox-arrow* :x-axis)) -1))
                              (send *target-bbox-arrow* :y-axis) (list2f (scale-vector (f2list (send *target-bbox-arrow* :y-axis)) -1))
                              ))
    (setq object-list (list *hironxjsk* *right-coords-arrow* *left-coords-arrow* *target-bbox* *target-bbox-arrow*))
    (objects object-list)
    )
  )

(defun detect-paper()
  (setq *paper-topic-name* "/paper_finder/output/box")
  (setq *target-paper* (find-paper *paper-topic-name* :condition #'valid-paper))
  (setq x-of-paper (x-of-cube *target-paper*))
  (setq y-of-paper (y-of-cube *target-paper*))
  (setq z-of-paper (z-of-cube *target-paper*))
  (setq *target-paper-arrow* (arrow))
  (send *target-paper-arrow* :newcoords (send *target-paper* :copy-worldcoords))
  (setq object-list (list *hironxjsk* *right-coords-arrow* *left-coords-arrow* *target-paper* *target-paper-arrow*))
  (objects object-list)
  )
