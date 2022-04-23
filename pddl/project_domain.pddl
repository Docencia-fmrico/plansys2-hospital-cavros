(define (domain project)
(:requirements :strips :equality :typing)

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

(:constants None - door)

;; Movement actions
(:action move
  :parameters (?from ?to - location ?robot - robot)
  :precondition 
    (and 
      (robot_at ?robot ?from)
      (connected ?from ?to ?door)  
    )
  :effect 
    (and 
      (robot_at ?robot ?to)
      (not (robot_at ?robot ?from))
    )
)

(:action move-to-zone
  :parameters (?room - room ?zone - zone ?robot - robot)
  :precondition 
    (and 
      (robot_at ?robot ?room)
      (in ?room ?zone)  
    )
  :effect 
    (and 
      (robot_at ?robot ?zone)
      (not (robot_at ?robot ?room))
    )
)

(:action leave-zone
  :parameters (?room - room ?zone - zone ?robot - robot)
  :precondition 
    (and 
      (robot_at ?robot ?zone)
      (in ?zone ?room)  
    )
  :effect 
    (and 
      (robot_at ?robot ?room)
      (not (robot_at ?robot ?zone))
    )
)

;; Object actions
(:action pick
  :parameters (?o - object ?l - location ?r - robot ?g - gripper)
  :precondition 
    (and
      (gripper_at ?g ?r)
      (object_at ?o ?l)
      (robot_at ?r ?l) 
      (gripper_free ?g)
    )
:effect 
  (and 
    (robot_carry ?r ?g ?o) 
    (not (object_at ?o ?l))
    (not (gripper_free ?g))
  )
)

(:action place
:parameters (?o - object ?l - location ?r - robot ?g - gripper)
:precondition 
  (and 
    (gripper_at ?g ?r)
    (robot_at ?r ?l)
    (robot_carry ?r ?g ?o)
  )
:effect 
  (and 
    (object_at ?o ?l)
    (gripper_free ?g)
    (not (robot_carry ?r ?g ?o))
  )
)
)
