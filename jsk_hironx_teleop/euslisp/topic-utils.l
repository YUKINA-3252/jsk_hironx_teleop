(defun one-shot-subscribe (topic-name mclass &key (timeout) (after-stamp) (unsubscribe t) (condition nil))
  "Subscribe message, just for once"
  (let (lmsg)
    (unless (ros::get-num-publishers topic-name)
      (cond
       (after-stamp
        (ros::subscribe topic-name mclass
                        #'(lambda (msg)
                            (let ((st (send msg :header :stamp)))
                              (when (> (send st :to-sec)
                                       (send after-stamp :to-sec))
                                (if condition
                                    (when (funcall condition msg)
                                      (setq lmsg msg))
                                    (setq lmsg msg)
                                    ))))))
       (t
        (ros::subscribe topic-name mclass
                        #'(lambda (msg)
                            (if condition
                                (when (funcall condition msg)
                                  (setq lmsg msg))
                                (setq lmsg msg)
                                ))))))
    (let ((finishtm (if timeout (ros::time-now))))
      (when finishtm
        (setq finishtm (ros::time+ finishtm (ros::time (/ timeout 1000.0)))))
      (while (not (and finishtm
                       (< (send (ros::time- finishtm (ros::time-now)) :to-Sec) 0)))
        (unix::usleep (* 50 1000))
        (ros::spin-once)
        (if lmsg (return))))
    (if unsubscribe (ros::unsubscribe topic-name))
    lmsg))
