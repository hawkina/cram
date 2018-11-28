;;; initializes the episode data by connecting to OpenEase and loads the Episode data within OpenEase
;;; contains paths to the Episode data and SemanticMap
;;; which might need to be adjusted, if other episode data is to be loaded. 

;;; care to give the right rcg stamp
;;; rcg_c is for finding out the displacement
;;; rcg_f has the to date better grasps
;;; eval2 has best full set pick and place
;;; rcg_d different grasps
(in-package :kvr)

;; TODO set a variable or something for the location of the episode data
(defun init-episode ()
  (ros-info (kvr) "initializing the episode data and connecting to database...")
  (start-ros-node "cram_knowrob_vr")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/home/hasu/ros_workspace/episode_data/episodes/Own-Episodes/set-clean-table/rcg_eval2/Episodes/")
  (owl-parse "/home/hasu/ros_workspace/episode_data/episodes/Own-Episodes/set-clean-table/rcg_eval2/SemanticMap.owl")
;  (owl-parse "/home/hasu/ros_workspace/episode_data/episodes/Own-Episodes/set-clean-table/rcg_eval2/iai-kitchen-knowledge.owl")
  (connect-to-db "Own-Episodes_set-clean-table")  
  (map-marker-init))


;; initializes the bullet world environment based on the bullet-world-tutorial
(defun init-bullet-world ()
  ;; append own meshes to meshes list so that they can be loaded.
  (btr:add-objects-to-mesh-list "cram_knowrob_vr")
  
  ;; init tf early. Otherwise there will be exceptions.
  (cram-tf::init-tf)
  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf prolog:*break-on-lisp-errors* t)

  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  
  ;; set costmap parameters
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 12 12))
    (prolog:<- (location-costmap:costmap-origin -6 -6))
    (prolog:<- (location-costmap:costmap-resolution 0.05))

    (prolog:<- (location-costmap:costmap-padding 0.2))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
    (prolog:<- (location-costmap:costmap-in-reach-distance 0.6))
    (prolog:<- (location-costmap:costmap-reach-minimal-distance 0.2)))

  ;;; initialization from the tutorial
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert
                    (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                     :normal (0 0 1) :constant 0))))
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:debug-window ?world)))

  ;;; load robot description
  (let ((robot-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param btr-belief:*robot-parameter*))))
    
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (cram-robot-interfaces:robot ?robot)
           (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
           (cram-robot-interfaces:robot-arms-carrying-joint-states ?robot ?joint-states)
           (assert (btr:joint-state ?world ?robot ?joint-states))
           (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))
;  (ros-info (kvr) "spawning urdf kitchen...")
 ;;; spawn kitchen as urdf
;  (let ((kitchen-urdf 
;         (cl-urdf:parse-urdf 
;           (roslisp:get-param btr-belief:*kitchen-parameter*))))
;    (prolog:prolog
;     `(and (btr:bullet-world ?world)
  (ros-info (kvr) "spawning semantic-map kitchen...")
;           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))

;(sem-map:get-semantic-map)
;(cram-occasions-events:clear-belief)

  (let ((kitchen-urdf 
          (cl-urdf:parse-urdf 
           (roslisp:get-param btr-belief:*kitchen-parameter*))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :semantic-map no-urdf-kitchen ((0 0 0) (0 0 0 1)))))))
                 ; :urdf ,kitchen-urdf)))))
  )

(defun init-items ()
  (ros-info (kvr) "spawning objects for experiments...")
  (add-bowl)
  (add-muesli)
  (add-axes)
  (add-fork)
  (add-cup)
  (add-milk)
  (add-spoon)
  (add-axes :axes2)
  (add-axes :axes3))

(defun init-full-simulation ()
  "Spawns all the objects which are necessary for the current
scenario (Meaning: Kitchen, Robot, Muesli, Milk, Cup, Bowl, Fork and 3 Axis
objects for debugging."
  (init-episode)
  (init-bullet-world)
  (init-items))

(defun prolog-testing()
  (cut:lazy-car 
   (json-prolog:prolog-simple
"map_name(MapName),
map_root_objects(MapName, Objects),
member(Obj, Objects),
owl_has(ObjInst, rdf:type, knowrob:'IslandArea'),
rdf_has(ObjInst, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', TP),
iri_xml_namespace(TP, PrefixType, Type),
iri_xml_namespace(ObjInst, PrefixName, Name),
owl_has(ObjInst, knowrob:pathToCadModel, Path),
object_mesh_path(ObjInst, ObjPath),
current_object_pose(ObjInst, Pose).")))
  
;  (print ?pose)
;  (length ?pose)
;  (destructuring-bind (a b c d) ?pose
;    (destructuring-bind (x y z) c
;      (destructuring-bind (w q1 q2 q3) d
;        (print c)
;        (print x)
;        (print y)
;        (print z))))
  

(defun prolog-testing2()
  (json-prolog:prolog-simple-1
   "map_name(MapName),
map_root_objects(MapName, Objects),
member(Obj, Objects),
rdf_has(Obj, 'http://knowrob.org/kb/knowrob_u.owl#attachedParent', Sub),
map_object_type(Sub, TP),
iri_xml_namespace(TP, PrefixType, Type),
iri_xml_namespace(Sub, PrefixName, Name)."
))

(defun prolog-testing3()
  (json-prolog:prolog-simple-1
   "object_mesh_path('http://knowrob.org/kb/unreal_log.owl#BaerenMarkeFrischeAlpenmilch18_4OVo', Pose)."
))

(defun prolog-testing4()
  (cut:var-value
   (intern "?OwlName")
   (cut:lazy-car 
    (json-prolog:prolog-simple
     "owl_has(OwlName, rdf:type, knowrob:'WasaDelicateCrisp')."))))

(defun prolog-testing5()
  (cut:lazy-car 
   (json-prolog:prolog
    `(and ("owl_has" "OwlName" "rdf:type" "knowrob:'IslandArea'"))
    :package :cram-knowrob-vr)))

(defun prolog-testing6()
  (json-prolog:prolog-simple-1 "map_name(MapName),
map_root_objects(MapName, Objects),
member(Obj, Objects),
current_object_pose(Obj, MeshPath)."))
