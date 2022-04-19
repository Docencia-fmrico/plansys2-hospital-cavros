(define (problem move-probl)
(:domain project)
(:objects 
    main s1 s2 z1 z2 z3 z4 g1 g2 g3 g4 g5 o1 o2 o3 o4 b1 b2 w1 - room
    corr1 corr2 corr3 - corridor
    object - object
    hand - gripper
    tiago - robot
)
(:init 
    ;; Initial state
    (robot_at tiago main)
    (gripper_at hand tiago)
    (gripper_free hand)    
    (object_at object w1)

    ;; Doors
    (open None)

    ;; Connections between every location
    ;; ;; MAIN
    (connected main corr1 None)
    (connected corr1 main None)
    (connected main corr2 None)
    (connected corr2 main None)
    (connected main corr3 None)
    (connected corr3 main None)

    (connected main s1 None)
    (connected s1 main None)
    (connected main s2 None)
    (connected s2 main None)

    (connected main z1 None)
    (connected z1 main None)
    (connected main z2 None)
    (connected z2 main None)
    (connected main z3 None)
    (connected z3 main None)
    (connected main z4 None)
    (connected z4 main None)

    (connected main g1 None)
    (connected g1 main None)
    (connected main g4 None)
    (connected g4 main None)

    (connected main o1 None)
    (connected o1 main None)
    (connected main o2 None)
    (connected o2 main None)

    (connected main b1 None)
    (connected b1 main None)
    (connected main b2 None)
    (connected b2 main None)

    ;; ;; CORR1
    (connected corr1 g2 None)
    (connected g2 corr1 None)
    (connected corr1 g3 None)
    (connected g3 corr1 None)
    (connected corr1 g5 None)
    (connected g5 corr1 None)

    (connected corr1 o3 None)
    (connected o3 corr1 None)
    (connected corr1 o4 None)
    (connected o4 corr1 None)

    (connected corr1 w1 None)
    (connected w1 corr1 None)
)

(:goal 
  (and 
    (object_at object main)
  )
)
)
