---- MODULE robots ----
EXTENDS TLC

CONSTANTS Robots, PossiblePositions
VARIABLES positions
vars == <<positions>>

Init == positions = [r \in Robots |-> "none"]

InCorridor(robot) == positions[robot] /= "none"

InSameCorridor(robot1, robot2) == 
    /\ InCorridor(robot1)
    /\ InCorridor(robot2)
    /\ positions[robot1] = positions[robot2]

StateValid ==
    /\ ~InSameCorridor("r1", "r2")
    /\ ~InSameCorridor("r1", "r3")
    /\ ~InSameCorridor("r2", "r3")

MoveRobot(robot, newPosition) ==
    /\ positions' = [positions EXCEPT ![robot] = newPosition]
    /\ StateValid'

Next ==
    \E pos \in PossiblePositions:
        \E r \in Robots:
            MoveRobot(r, pos)
    /\ PrintT(positions)

Spec == 
    Init 
    /\ [][Next]_vars 

TypeOK == positions \in [Robots -> PossiblePositions]
THEOREM Spec => []TypeOK
====