;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :bullet-reasoning)

(def-fact-group pose-sampling ()
  (<- (random-poses-on ?bottom ?top ?poses)
    (ground (?bottom ?top))
    (not (bound ?poses))
    (generate ?poses (random-poses-on ?bottom ?top)))

  (<- (random-poses-on ?n ?bottom ?top ?poses)
    (ground (?bottom ?top))
    (not (bound ?poses))
    (generate ?tmp (random-poses-on ?bottom ?top))
    (take ?n ?tmp ?poses))

  (<- (n-poses-on ?n ?bottom ?top ?poses)
    (ground (?bottom ?top ?n))
    (not (bound ?poses))
    (generate ?poses (n-poses-on ?bottom ?top ?n)))

  (<- (desig-poses ?n ?desig ?obj-name ?poses)
    (ground (?desig ?obj-name ?n))
    (location-costmap:merged-desig-costmap ?desig ?cm)
    (location-costmap:costmap-samples ?cm ?desig-poses)
    (bullet-world ?w)
    (%object ?w ?obj-name ?obj)
    (lisp-fun obj-poses-on ?obj ?desig-poses ?poses-inf)
    (take ?n ?poses-inf ?poses)))