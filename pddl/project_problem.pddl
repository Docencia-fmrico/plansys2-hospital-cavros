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


    ;; Connections between every location
    ;; ;; MAIN
    (connected main corr1)
    (connected corr1 main)
    (connected main corr2)
    (connected corr2 main)
    (connected main corr3)
    (connected corr3 main)

    (connected main s1)
    (connected s1 main)
    (connected main s2)
    (connected s2 main)

    (connected main z1)
    (connected z1 main)
    (connected main z2)
    (connected z2 main)
    (connected main z3)
    (connected z3 main)
    (connected main z4)
    (connected z4 main)

    (connected main g1)
    (connected g1 main)
    (connected main g4)
    (connected g4 main)

    (connected main o1)
    (connected o1 main)
    (connected main o2)
    (connected o2 main)

    (connected main b1)
    (connected b1 main)
    (connected main b2)
    (connected b2 main)

    ;; ;; CORR1
    (connected corr1 g2)
    (connected g2 corr1)
    (connected corr1 g3)
    (connected g3 corr1)
    (connected corr1 g5)
    (connected g5 corr1)

    (connected corr1 o3)
    (connected o3 corr1)
    (connected corr1 o4)
    (connected o4 corr1)

    (connected corr1 w1)
    (connected w1 corr1)
)

(:goal 
  (and 
    (object_at object main)
  )
)
)
