(define (domain project)
(:requirements :strips :equality :typing :durative-actions :fluents)

(:types
    room zone corridor - location
    robot
    gripper  
    object
)

(:predicates  
  ;; Locations
  (in ?l1 ?l2 - location)
  (connected ?l1 ?l2 - location)

  ;; Robot
  (robot_at ?r - robot ?l - location)
  (gripper_free ?g - gripper)
  (gripper_at ?g - gripper ?r - robot)
  (robot_carry ?r - robot ?g - gripper ?o - object)

  ;; Object
  (object_at ?o - object ?l - location)
)

;; Movement actions
(:durative-action move
  :parameters (?from ?to - location ?robot - robot)
  :duration (= ?duration 1)
  :condition 
    (and 
      (at start (robot_at ?robot ?from))
      (at start (connected ?from ?to))
    )
  :effect 
    (and 
      (at end (robot_at ?robot ?to))
      (at start (not (robot_at ?robot ?from)))
    )
)

(:durative-action move-to-zone
  :parameters (?room - room ?zone - zone ?robot - robot)
  :duration (= ?duration 1)
  :condition 
    (and 
      (at start (robot_at ?robot ?room))
      (at start (in ?room ?zone))
    )
  :effect 
    (and 
      (at end (robot_at ?robot ?zone))
      (at start (not (robot_at ?robot ?room)))
    )
)

(:durative-action leave-zone
  :parameters (?room - room ?zone - zone ?robot - robot)
  :duration (= ?duration 1)
  :condition 
    (and 
      (at start (robot_at ?robot ?zone))
      (at start (in ?zone ?room))
    )
  :effect 
    (and 
      (at end (robot_at ?robot ?room))
      (at start (not (robot_at ?robot ?zone)))
    )
)

;; Object actions
(:durative-action pick
  :parameters (?o - object ?l - location ?r - robot ?g - gripper)
  :duration (= ?duration 1)
  :condition 
    (and
      (at start (gripper_at ?g ?r))
      (at start (object_at ?o ?l))
      (at start (robot_at ?r ?l))
      (at start (gripper_free ?g))
    )
  :effect 
    (and 
      (at end (robot_carry ?r ?g ?o))
      (at start (not (object_at ?o ?l)))
      (at start (not (gripper_free ?g)))
    )
)

(:durative-action place
  :parameters (?o - object ?l - location ?r - robot ?g - gripper)
  :duration (= ?duration 1)
  :condition 
    (and 
      (at start (gripper_at ?g ?r))
      (at start (robot_at ?r ?l))
      (at start (robot_carry ?r ?g ?o))
    )
  :effect 
    (and 
      (at end (object_at ?o ?l))
      (at end (gripper_free ?g))
      (at start (not (robot_carry ?r ?g ?o)))
    )
  )
)
