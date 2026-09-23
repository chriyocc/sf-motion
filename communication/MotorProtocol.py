from SFMotionCom import SFMotion


class MotorProtocol:
    """Compatibility API for the custom plotter using the current SFM protocol."""

    _channel_registers = {
        "ia": "ia",
        "ib": "ib",
        "ic": "ic",
        "i_alpha": "i_alpha",
        "i_beta": "i_beta",
        "id": "id",
        "iq": "iq",
        "va": "va",
        "vb": "vb",
        "vc": "vc",
        "v_alpha": "v_alpha",
        "v_beta": "v_beta",
        "vd": "vd",
        "vq": "vq",
        "e_rad": "e_rad",
        "actual_rpm": "actual_rpm",
        "actual_angle": "actual_angle",
        "Is_ref": "is_ref",
        "rpm_ref": "rpm_ref",
        "pos_ref": "pos_ref",
        "m_angle_rad": "m_angle_rad",
        "m_angle_rad_comp": "m_angle_rad_comp",
        "v_bus": "v_bus",
    }

    def __init__(self, serial_conn=None, acq_thread=None, config_file=None):
        self.protocol = SFMotion(serial_conn, acq_thread)
        self.ser = serial_conn
        self.acq_thread = acq_thread
        self.plotter_channels = []
        register_addresses = {
            register["name"]: address
            for address, register in self.protocol.registers.items()
        }
        self.plotter_dict = {
            name: register_addresses[register_name]
            for name, register_name in self._channel_registers.items()
        }
        self._address_to_channel = {address: name for name, address in self.plotter_dict.items()}

    def _get_value(self, name):
        return getattr(self.protocol, f"get_{name}")()

    def _set_value(self, name, value):
        return getattr(self.protocol, f"set_{name}")(value)

    def _get_group(self, names):
        return {key: self._get_value(register) for key, register in names.items()}

    def _set_group(self, names, values):
        addresses = {r["name"]: addr for addr, r in self.protocol.registers.items()}
        for key, register in names.items():
            self.protocol.validate_write(addresses[register], values[key])
        try:
            for key, register in names.items():
                self._set_value(register, values[key])
        except Exception as exc:
            raise RuntimeError(
                "Parameter group may be partially applied. Reconnect if required, "
                "then read all parameters before retrying."
            ) from exc
        return True

    def get_actual_angle(self):
        return self._get_value("actual_angle")

    def get_foc_mode(self):
        return {"mode": self._get_value("foc_mode")}

    def set_foc_mode(self, mode):
        return self._set_value("foc_mode", mode)

    def get_foc_motor_mode(self):
        return {"mode": self._get_value("motor_mode")}

    def set_foc_motor_mode(self, mode):
        return self._set_value("motor_mode", mode)

    def set_foc_current_set_point(self, value):
        return self._set_value("current_set_point", value)

    def set_foc_speed_set_point(self, value):
        return self._set_value("speed_set_point", value)

    def set_foc_position_set_point(self, value):
        return self._set_value("position_set_point", value)

    def set_pole_pairs(self, value):
        return self._set_value("pole_pairs", value)

    def get_pole_pairs(self):
        return {"value": self._get_value("pole_pairs")}

    def get_pid_id(self):
        return self._get_group({"kp": "id_kp", "ki": "id_ki", "deadband": "id_deadband"})

    def set_pid_id(self, kp, ki, deadband):
        return self._set_group({"kp": "id_kp", "ki": "id_ki", "deadband": "id_deadband"},
                               {"kp": kp, "ki": ki, "deadband": deadband})

    def get_pid_iq(self):
        return self._get_group({"kp": "iq_kp", "ki": "iq_ki", "deadband": "iq_deadband"})

    def set_pid_iq(self, kp, ki, deadband):
        return self._set_group({"kp": "iq_kp", "ki": "iq_ki", "deadband": "iq_deadband"},
                               {"kp": kp, "ki": ki, "deadband": deadband})

    def get_pid_speed(self):
        return self._get_group({"kp": "speed_kp", "ki": "speed_ki", "out_max": "speed_out_max",
                                "deadband": "speed_deadband"})

    def set_pid_speed(self, kp, ki, out_max, deadband):
        return self._set_group(
            {"kp": "speed_kp", "ki": "speed_ki", "out_max": "speed_out_max",
             "deadband": "speed_deadband"},
            {"kp": kp, "ki": ki, "out_max": out_max, "deadband": deadband},
        )

    def get_pid_position(self):
        return self._get_group({"kp": "position_kp", "ki": "position_ki", "kd": "position_kd",
                                "out_max": "position_out_max", "deadband": "position_deadband",
                                "d_fc": "position_d_filter_fc"})

    def set_pid_position(self, kp, ki, kd, out_max, deadband, d_fc):
        return self._set_group(
            {"kp": "position_kp", "ki": "position_ki", "kd": "position_kd",
             "out_max": "position_out_max", "deadband": "position_deadband",
             "d_fc": "position_d_filter_fc"},
            {"kp": kp, "ki": ki, "kd": kd, "out_max": out_max,
             "deadband": deadband, "d_fc": d_fc},
        )

    def get_field_weakening(self):
        return self._get_group({"kp": "fw_kp", "ki": "fw_ki", "out_min": "fw_out_min"})

    def set_field_weakening(self, kp, ki, out_min):
        return self._set_group({"kp": "fw_kp", "ki": "fw_ki", "out_min": "fw_out_min"},
                               {"kp": kp, "ki": ki, "out_min": out_min})

    def get_field_weakening_enable(self):
        return {"enable": self._get_value("fw_enable")}

    def set_field_weakening_enable(self, enable):
        return self._set_value("fw_enable", int(bool(enable)))

    def get_mtpa_enable(self):
        return {"enable": self._get_value("mtpa_enable")}

    def set_mtpa_enable(self, enable):
        return self._set_value("mtpa_enable", int(bool(enable)))

    def plotter_add_line(self, channel):
        if channel not in self.plotter_dict:
            raise ValueError(f"Unknown plot channel: {channel}")
        if channel not in self.plotter_channels:
            self.protocol.enable_streaming(self.plotter_dict[channel])
            self.plotter_channels.append(channel)
        return True

    def plotter_remove_line(self, channel):
        if channel in self.plotter_channels:
            self.protocol.disable_streaming(self.plotter_dict[channel])
            self.plotter_channels.remove(channel)
        return True

    def plotter_remove_all_line(self):
        for channel in list(self.plotter_channels):
            self.plotter_remove_line(channel)
        return True

    def reset_plotter_streams(self):
        for address, register in self.protocol.registers.items():
            if not register["streaming"]:
                continue
            try:
                self.protocol.disable_streaming(address)
            except RuntimeError as exc:
                if str(exc) != "Device returned error: -1":
                    raise
        self.plotter_channels.clear()

    def channel_name(self, address):
        return self._address_to_channel.get(address)

    def save_config(self):
        return self.protocol.set_save_config()

    def set_default_config(self):
        return self.protocol.set_default_config()

    def self_commissioning(self):
        return self.protocol.self_commissioning()

    def close(self):
        self.protocol.close()

    def __getattr__(self, name):
        scalar_registers = {
            "kv": "kv",
            "rs": "rs",
            "ld": "ld",
            "lq": "lq",
            "flux_linkage": "flux_linkage",
        }
        if name.startswith(("get_", "set_")):
            operation, key = name.split("_", 1)
            register = scalar_registers.get(key)
            if register:
                if operation == "get":
                    return lambda: {"value": self._get_value(register)}
                return lambda value: self._set_value(register, value)
        raise AttributeError(name)
