import moveit_commander
import time

if __name__ == "__main__":
    move = moveit_commander.MoveGroupCommander('asp_arm')
    move.set_joint_value_target([  1.6,  0.01,  0.0, 1.09, 1.90])
    before = time.time()
    move.go()
    after = time.time()
    print after - before
