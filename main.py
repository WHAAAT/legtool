import leg_control.example_settings as leg_control
leg_control = leg_control.example_settings()

# set command translate 1000 mm to y direktion with 0 degree roll and pitch
translate_x = 0
translate_y = 1000
body_pitch_deg = 0
body_roll_deg = 0
leg_control.test_ripple_basic(translate_x, translate_y, body_pitch_deg, body_roll_deg)


def main():
    #advance 3 steps
    for i in range(3):
        leg_poses = leg_control.getMovement(i)
        move_from = leg_poses[0]
        move_to = leg_poses[1]
        print "move_from: leg 1",move_from[0],"leg 2",move_from[1],"leg 3",move_from[2],"leg 4",move_from[3]
        print "move_to  : leg 1",move_to[0],"leg 2",move_to[1],"leg 3",move_to[2],"leg 4",move_to[3]
        print ""


if __name__ == '__main__':
    main()
