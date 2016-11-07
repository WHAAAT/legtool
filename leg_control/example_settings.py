import ripple
import leg_ik
import common
import copy

FRAME_ROBOT, FRAME_WORLD, FRAME_BODY = range(3)
PROJECTION_XY, PROJECTION_YZ, PROJECTION_XZ = range(3)


class example_settings(object):
    def test_ripple_basic(self, translate_x_mm, translate_y_mm, body_pitch_deg, body_roll_deg):
        self.config = ripple.RippleConfig()

        mounts = [(50., 50.), (50., -50.), (-50., -50.), (-50., 50.)]

        ik_config = leg_ik.Configuration()
        ik_config.coxa_min_deg = -85.0
        ik_config.coxa_idle_deg = 0.0
        ik_config.coxa_max_deg = 85.0
        ik_config.coxa_length_mm = 20.0
        ik_config.femur_min_deg = -85.0
        ik_config.femur_idle_deg = 0.0
        ik_config.femur_max_deg = 85.0
        ik_config.femur_length_mm = 70.0
        ik_config.tibia_min_deg = -85.0
        ik_config.tibia_idle_deg = 0.0
        ik_config.tibia_max_deg = 85.0
        ik_config.tibia_length_mm = 100.0
        ik_config.servo_speed_dps = 360
        ik_config.femur_sign = -1
        ik_config.tibia_sign = -1
        ik_config.coxa_sign = -1

        for leg_num in range(4):
            leg = common.LegConfig()

            leg.mount_x_mm = mounts[leg_num][0]
            leg.mount_y_mm = mounts[leg_num][1]
            leg.mount_z_mm = 0.0

            leg.idle_x_mm = 85.0
            leg.idle_y_mm = 85.0
            leg.idle_z_mm = 0.0

            this_ik_config = copy.copy(ik_config)
            this_ik_config.coxa_ident = leg_num * 3 + 0
            this_ik_config.femur_ident = leg_num * 3 + 1
            this_ik_config.tibia_ident = leg_num * 3 + 2
            if leg_num == 0 or leg_num == 2:
                this_ik_config.coxa_max_deg = 85
                this_ik_config.coxa_min_deg = -85
            if leg_num == 1 or leg_num == 3:
                this_ik_config.coxa_max_deg = 85
                this_ik_config.coxa_min_deg = -85

            leg.leg_ik = leg_ik.LizardIk(this_ik_config)

            self.config.mechanical.leg_config[leg_num] = leg

        self.config.mechanical.body_cog_x_mm = 0.0
        self.config.mechanical.body_cog_y_mm = 0.0
        self.config.mechanical.body_cog_z_mm = 0.0
        self.config.swing_percent = 70.0
        self.config.statically_stable = True
        self.config.max_cycle_time_s = 4.0

        self.config.lift_height_mm = 35.0
        # config.min_swing_percent = 50.0
        # config.max_swing_percent = 70.0
        self.config.position_margin_percent = 80
        self.config.leg_order = [0, 1, 2, 3]
        self.config.body_z_offset_mm = 60.0
        self.config.statically_stable = True
        self.config.static_center_factor = 3.0
        self.config.static_margin_mm = 35.0
        self.config.static_stable_factor = 10.0

        self.config.statically_stable = True

        self.command = ripple.Command()

        self.config.max_cycle_time_s = 4

        self.command.translate_y_mm_s = translate_y_mm
        self.command.translate_x_mm_s = translate_x_mm

        self.command.rotate_deg_s = 0.0
        self.command.body_x_mm = 0.0
        self.command.body_y_mm = 0.0
        self.command.body_z_mm = 0.0
        self.command.body_pitch_deg = body_pitch_deg
        self.command.body_roll_deg = body_roll_deg
        self.command.body_yaw_deg = 0.0

        self.gait = ripple.RippleGait(self.config)

    def getMovement(self, index):
        self.gait.cycle_time_s = 4.0
        # advance = 40.0 / 1000.0
        # begin_state = self.gait.advance_time(advance)
        begin_state = self.get_start_state(self.config, self.command, index)
        begin_state = self.gait.set_state(begin_state, self.command)
        current_states = ([begin_state.copy()] + [self.gait.advance_phase(0.01).copy() for x in range(1)])
        points = []
        for i in range(len(current_states)):
            leg = current_states[i]
            leg_1 = self._project(FRAME_BODY, leg.legs[0].point, leg.legs[0].frame, leg)
            leg_2 = self._project(FRAME_BODY, leg.legs[3].point, leg.legs[3].frame, leg)
            leg_3 = self._project(FRAME_BODY, leg.legs[1].point, leg.legs[1].frame, leg)
            leg_4 = self._project(FRAME_BODY, leg.legs[2].point, leg.legs[2].frame, leg)
            points.append([leg_1, leg_2, leg_3, leg_4])
        return points

    def _project(self, current_frame, point, frame, state):
        target_frame = None
        if current_frame == FRAME_ROBOT:
            target_frame = state.robot_frame
        elif current_frame == FRAME_WORLD:
            target_frame = state.world_frame
        elif current_frame == FRAME_BODY:
            target_frame = state.body_frame

        target_point = target_frame.map_from_frame(frame, point)

        return [target_point.x, target_point.y, target_point.z]

    def get_start_state(self, ripple_config, command, index):
        begin_index = index
        this_ripple = ripple.RippleGait(ripple_config)

        if begin_index == 0:  # Idle

            begin_state = this_ripple.get_idle_state()
        else:
            begin_state = this_ripple.get_idle_state()
            this_ripple.set_state(begin_state, command)

            for x in range(index):
                begin_state = this_ripple.advance_phase(0.01)

            begin_state.phase = 0.0

        return begin_state


if __name__ == "__main__":
    foo = main_test()
