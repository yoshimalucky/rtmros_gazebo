(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")


(defun samplerobot-init-ex
  (&key (real t))
  (cond
   (real
    (samplerobot-init))
   (t
    (if (not (boundp '*sr*))
	(setq *sr* (instance hrp2jsknt-robot :init)))))
  (setq *robot* *sr*)
  (objects (list *sr*))
  (send *irtviewer* :change-background #f(1 1 1)))

(defun init
  (&key (real t))
  (samplerobot-init-ex :real real))

(defun reset-pose
  (&key (wait-time 2000) (wait? t))
  (send-special-pose :reset-pose :wait-time wait-time :wait? wait?))

(defun reset-manip-pose
  (&key (wait-time 2000) (wait? t))
  (send-special-pose :reset-manip-pose :wait-time wait-time :wait? wait?))

(defun send-special-pose
  (pose &key (wait-time 2000) (wait? t))
  (send *ri* :angle-vector (send *robot* pose) wait-time)
  (when wait? (send *ri* :wait-interpolation)))

(defun model2real
  ()
  (send *ri* :angle-vector (send *robot* :angle-vector)))
