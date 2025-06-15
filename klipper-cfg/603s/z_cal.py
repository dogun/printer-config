from mcu import MCU_endstop

class ZCalHelper:
    def __init__(self, config):
        self.z_offset = config.getfloat('z_offset', 0.1, above=0.)
        
        self.tolerance = config.getfloat('tolerance', 0.04, above=0.01)
        self.retries = config.getint('retries', 5, minval=2)
        self.samples = config.getint('samples', 4, minval=3)
        
        self.position_min = config.getfloat('position_min', -5.0, above=-10.)
        self.z_speed = config.getint('z_speed', 25, minval=10)
        self.lift_speed = config.getint('lift_speed', 25, minval=5)
        self.retract_dist = config.getfloat('retract_dist', 20.0, above=5.)
        self.probe_x = config.getfloat('probe_x', 140.0, above=50.)
        self.probe_y = config.getfloat('probe_y', 90.0, above=50.)
        
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self.handle_home_rails_end)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('CAL_Z', self.cmd_CAL_Z,
                                    desc=self.cmd_CAL_Z_help)
        self.gcode_move = self.printer.lookup_object('gcode_move')
    def handle_connect(self):
        pass
    def handle_home_rails_end(self, homing_state, rails):
        pass
    cmd_CAL_Z_help = ("Cal z")
    def cmd_CAL_Z(self, gcmd):
        gcmd.respond_info("start Cal z")
        gcmd.respond_info("config: z_offset:%.3f tolerance:%.3f retries:%d samples:%d position_min:%.3f z_speed:%d lift_speed:%d retract_dist:%.3f probe_x:%.3f probe_y:%.3f" % (self.z_offset, self.tolerance, self.retries, self.samples, self.position_min, self.z_speed, self.lift_speed, self.retract_dist, self.probe_x, self.probe_y))
        probe_zero = self._probe_z(gcmd)
        gcmd.respond_info("probe_zero: %.4f" % probe_zero)
        offset = probe_zero + self.z_offset
        self._set_new_gcode_offset(offset)
        gcmd.respond_info("end Cal z, offset: %.4f" % offset)
    def _probe_z(self, gcmd):
        retries = 0
        positions = []
        while len(positions) < self.samples:
            curpos = self._probe()
            gcmd.respond_info("probe: %.4f" % curpos[2])
            positions.append(curpos[:3])
            z_positions = [p[2] for p in positions]
            if max(z_positions) - min(z_positions) > self.tolerance:
                if retries >= self.retries:
                    raise gcmd.error("Probe samples exceed tolerance")
                gcmd.respond_info("Probe samples exceed tolerance."
                                       " Retrying...")
                retries += 1
                positions = []
        return self._calc_mean(positions)[2]
    def _set_new_gcode_offset(self, offset):
        gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET",
                                                      "SET_GCODE_OFFSET",
                                                      {'Z': offset})
        self.gcode_move.cmd_SET_GCODE_OFFSET(gcmd_offset)
    def _probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[2] = self.position_min
        phoming = self.printer.lookup_object('homing')
        probe = self.printer.lookup_object('probe')
        self._move([self.probe_x, self.probe_y, None], self.lift_speed)
        curpos = phoming.probing_move(probe.mcu_probe, pos, self.z_speed)
        #curpos = [180, 90, 0.1]
        self._move([None, None, curpos[2] + self.retract_dist], self.lift_speed)
        return curpos
    def _move(self, coord, speed):
        self.printer.lookup_object('toolhead').manual_move(coord, speed)
    def _calc_mean(self, positions):
        count = float(len(positions))
        return [sum([pos[i] for pos in positions]) / count
                for i in range(3)]
def load_config(config):
    return ZCalHelper(config)
