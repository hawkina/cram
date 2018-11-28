;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :btr)

(defclass simple-semantic-map-object (semantic-map-object) ())

(defmethod initialize-instance :after ((semantic-map-object simple-semantic-map-object)
                                       &key (collision-group :default-filter)
                                         (collision-mask :all-filter)
                                         (pose (cl-transforms:make-identity-pose)))
  (roslisp:ros-warn (btr) "init-instance-simple. Params:
 ~a
~a
~a
~a" semantic-map-object collision-group collision-mask pose)
  (with-slots (pose-reference-body semantic-map) semantic-map-object
    
    (roslisp:ros-warn (btr) "parts: ~a " (sem-map-utils:semantic-map-parts  semantic-map :recursive t)))
  
  (with-slots (pose-reference-body semantic-map) semantic-map-object
    (let ((bodies (loop for part in (sem-map-utils:semantic-map-parts
                                     semantic-map :recursive t)
                        for body = (semantic-map-part-rigid-body
                                    part
                                    :pose pose
                                    :collision-group collision-group
                                    :collision-mask collision-mask
                                    :mesh t) ;TODO
                        when body collect body)
                  (roslisp:ros-warn (btr) "done loading parts")))
      (initialize-rigid-bodies semantic-map-object bodies)
      (setf pose-reference-body (car bodies)))))

(defmethod copy-object ((obj simple-semantic-map-object) (world bt-reasoning-world))
  (with-slots (semantic-map) obj
    (change-class
     (call-next-method) 'simple-semantic-map-object
     :semantic-map (sem-map-utils:copy-semantic-map-object semantic-map))))

(defgeneric semantic-map-part-rigid-body (part &key pose collision-group collision-mask mesh)
  (:documentation "Returns a rigid body for the semantic map part `part'.")
;  (:method ((part sem-map-utils:semantic-map-geom) &key pose collision-group collision-mask (mesh nil))
;    (roslisp:ros-warn (btr) "this is called")
;    (make-instance 'rigid-body
;      :name (intern (sem-map-utils:name part))
;      :group collision-group
;      :mask collision-mask
;      :pose (cl-transforms:transform-pose
;             (cl-transforms:pose->transform pose)
;             (sem-map-utils:pose part))
;      :collision-shape (make-instance 'box-shape
;                         :half-extents (cl-transforms:v*
;                                        (sem-map-utils:dimensions part)
;                                        0.5))))
  
  (:method ((part t) &key pose collision-group collision-mask (mesh nil))
    (declare (ignore pose collision-group collision-mask))
    (warn 'simple-warning
          :format-control "Unable to generate a rigid body for semantic map part of type `~a'."
          :format-arguments (list (type-of part))))
;;TODO
  (:method ((part sem-map-utils:semantic-map-part) &key pose collision-group collision-mask (mesh t))
    (roslisp:ros-warn (btr) "spawning rigid body based on mesh for semantic map part: ~a" part)
    (if (eql part nil)
        (roslisp:ros-warn (btr) "Part is NIL")
        ;;NOTE Step1: load mesh
        (if (eql pose '1)
            (roslisp:ros-warn (btr) "No Pose Value")
            (progn    
              (roslisp:ros-warn (btr) "Pose: ~a"   (sem-map-utils::get-mesh-pose (sem-map-utils:owl-name part)))
              (let* ((mesh-name (sem-map-utils:owl-name part))
                     (mesh-model nil))
                (setf mesh-model (physics-utils:scale-3d-model
                                  (let ((uri
                                          (physics-utils:parse-uri
                                           (remove #\'
                                                   (symbol-name
                                                    (sem-map-utils::get-mesh-path mesh-name)))))
                                        (model nil))
                                    (roslisp:ros-warn (btr) "uri: ~a" uri)
                                    (physics-utils:load-3d-model uri :flip-winding-order t
                                                                     :compound model)) 
                                  1.0)) ;scaling factor is currently 1.0 anyway in every model
      
                (roslisp:ros-warn (btr) "part name: ~a" (intern (sem-map-utils:name part)))
      
            
            ;;NOTE Step2: spawn model or create instance of it
                (roslisp:ros-warn (btr) "Rigid Body" )
                (make-instance 'rigid-body
                  :name (intern (sem-map-utils:name part))
                  :mass 0.5
                  :group collision-group
                  :mask collision-mask
                  :pose  (cl-transforms:transform-pose
                          (cl-transforms:pose->transform pose)
                          (sem-map-utils:pose part))
                  :collision-shape (make-instance 'convex-hull-mesh-shape
                                     :points (physics-utils:3d-model-vertices mesh-model)
                                     :faces (physics-utils:3d-model-faces mesh-model)
                                     :color '(0.5 0.5 0.5 1.0) ;some random color
                                     :disable-face-culling t)))
              (roslisp:ros-warn (btr) "Done!"))))))

