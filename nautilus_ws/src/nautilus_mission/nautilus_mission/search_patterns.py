from enums import ObjectID

def search_bottom_spiral(self):
    now = self.node.get_clock().now()

    elapsed = (now - self.bottom_search_leg_start).nanoseconds / 1e9

    base_time = 0.5
    step_time = 0.1

    leg_time = base_time + step_time * (self.bottom_search_leg // 2)

    if elapsed > leg_time:
        self.bottom_search_leg += 1
        self.bottom_search_leg_start = now

    dirs = [
        (1520, 1500),  # forward
        (1500, 1520),  # strafe right
        (1480, 1500),  # backward
        (1500, 1480),  # strafe left
    ]

    forward_cmd, lateral_cmd = dirs[self.bottom_search_leg % 4]

    self.node.publish_cmd("forward",forward_cmd)
    self.node.publish_cmd("lateral",lateral_cmd)

def forward_search(self):
    gate_like_ids = [ObjectID.GATE_MID_RIGHT, ObjectID.GATE_LEFT_MID]
    gate_id = next((id for id in self.target_ids if id in gate_like_ids), None)
    spin_amplitude = abs(self.current_objective.search.spin_pwm-1500)
    if gate_id is not None:
        if gate_id == ObjectID.GATE_LEFT_MID:
            if self.is_target_present([ObjectID.GATE_LEG_L]):
                cmd = 1500 + spin_amplitude
            elif self.is_target_present([ObjectID.GATE_LEG_CENTER]):
                cmd = 1500 - spin_amplitude
            else:
                cmd = self.current_objective.search.spin_pwm

        elif gate_id == ObjectID.GATE_MID_RIGHT:
            if self.is_target_present([ObjectID.GATE_LEG_CENTER]):
                cmd = 1500 + spin_amplitude
            elif self.is_target_present([ObjectID.GATE_LEG_R]):
                cmd = 1500 - spin_amplitude
            else:
                cmd = self.current_objective.search.spin_pwm

    else:
        cmd = self.current_objective.search.spin_pwm

    self.node.publish_cmd("yaw",cmd)