;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :pp-plans)

(defmethod man-int:get-action-trajectory ((action-type (eql :picking-up))
                                          arm
                                          grasp
                                          objects-acted-on
                                          &key)
  (let* ((object (car objects-acted-on))
         (object-name (desig:desig-prop-value object :name))
         (object-type (desig:desig-prop-value object :type))
         (object-transform (man-int:get-object-transform object))
         (object-to-standard-gripper->base-to-particular-gripper
            (man-int:make-object-to-standard-gripper->base-to-particular-gripper-transformer
             object-transform arm))
         (grasp-transform
           (cram-manipulation-interfaces:get-object-type-to-gripper-transform
            object-type object-name arm grasp)))
    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar object-to-standard-gripper->base-to-particular-gripper
                              transforms)))
            (list
             :reaching
             :grasping
             :lifting)
            (list
             (list
              (man-int:get-object-type-to-gripper-pregrasp-transform
               object-type object-name arm grasp grasp-transform)
              (man-int:get-object-type-to-gripper-2nd-pregrasp-transform
               object-type object-name arm grasp grasp-transform))
             (list grasp-transform)
             (list
              (man-int:get-object-type-to-gripper-lift-transform
               object-type object-name arm grasp grasp-transform)
              (man-int:get-object-type-to-gripper-2nd-lift-transform
               object-type object-name arm grasp grasp-transform))))))

(defmethod man-int:get-action-trajectory ((action-type (eql :placing))
                                          arm
                                          grasp
                                          objects-acted-on
                                          &key
                                            target-object-transform)
  (let* ((object (car objects-acted-on))
         (object-name (desig:desig-prop-value object :name))
         (object-type (desig:desig-prop-value object :type))
         (object-to-standard-gripper->base-to-particular-gripper
           (man-int:make-object-to-standard-gripper->base-to-particular-gripper-transformer
            target-object-transform arm))
         (grasp-transform
           (cram-manipulation-interfaces:get-object-type-to-gripper-transform
            object-type object-name arm grasp)))
    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar object-to-standard-gripper->base-to-particular-gripper
                              transforms)))
            (list
             :reaching
             :putting
             :retracting)
            (list
             (list
              (man-int:get-object-type-to-gripper-lift-transform
               object-type object-name arm grasp grasp-transform)
              (man-int:get-object-type-to-gripper-2nd-lift-transform
               object-type object-name arm grasp grasp-transform))
             (list grasp-transform)
             (list
              (man-int:get-object-type-to-gripper-pregrasp-transform
               object-type object-name arm grasp grasp-transform)
              (man-int:get-object-type-to-gripper-2nd-pregrasp-transform
               object-type object-name arm grasp grasp-transform))))))