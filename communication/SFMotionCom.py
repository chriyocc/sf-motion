import struct
import math
import queue
import threading
import serial
import time
import json
import numpy as np
from enum import IntEnum
from pathlib import Path


class SFMComType(IntEnum):
    RESPONSE = 0x00
    READ = 0x01
    WRITE = 0x02
    ENABLE_STREAMING = 0x03
    DISABLE_STREAMING = 0x04
    STREAMING = 0x05
    EVENT = 0x06


class SFMotion:
    HEADER = b"\xA5\xA5"

    def __init__(self, serial_conn=None, acq_thread=None, json_path=None):
        self._request_lock = threading.Lock()
        self._desynchronized = False
        self.ser = serial_conn
        self.acq_thread = acq_thread

        if json_path is None:
            json_path = Path(__file__).with_name("SFMotionRegisters.json")
        with open(json_path, "r") as f:
            config = json.load(f)

        self.registers = {
            item["address"]: item
            for item in config["registers"]
        }
        self.channel = IntEnum(
            "channel",
            {
                item["name"]: item["address"]
                for item in config["registers"]
                if item["streaming"]
            }
        )
        self._create_api()

    def _create_api(self):
        for register in self.registers.values():
            address = register["address"]
            name = register["name"]
            if register["read"]:
                setattr(self, f"get_{name}", 
                    lambda address=address: self.read(address)
                )
            if register["write"]:
                setattr(self, f"set_{name}",
                    lambda value=None, address=address: self.write(address, value)
                )

    def close(self):
        self.ser.close()

    # ------------------------------------------------------------
    # Low-level communication
    # ------------------------------------------------------------

    def _send_frame(self, com_type, address, data=b""):
        payload = bytes([com_type, address]) + data
        frame = (self.HEADER + bytes([len(payload)]) + payload)
        # print(f"TX: {frame.hex(' ')}")
        self.ser.write(frame)

    def validate_write(self, address, value):
        register = self.registers.get(address)
        if register is None or not register["write"]:
            raise ValueError(f"Address {address} is not writable")
        name = register["name"]
        if name.startswith(("id_", "iq_", "speed_", "position_", "fw_")) and name.endswith(
            ("_kp", "_ki", "_kd", "_deadband", "_out_max", "_out_min", "_d_filter_fc")
        ):
            valid = math.isfinite(value)
            if name.endswith("_out_min"):
                valid = valid and value <= 0
            elif name.endswith("_d_filter_fc"):
                valid = valid and value > 0
            else:
                valid = valid and value >= 0
            if not valid:
                raise ValueError(f"Invalid PID value for {name}: {value}")
        if register["format"] is not None:
            struct.pack(register["format"], value)

    def _exchange(self, com_type, address, data=b"", expected_size=1):
        # Only one outstanding request: the wire protocol has no transaction ID.
        with self._request_lock:
            disable = (com_type == SFMComType.WRITE and
                       self.registers[address]["name"] == "motor_mode" and data == b"\x06")
            if self._desynchronized:
                if disable:
                    self._send_frame(com_type, address, data)
                    raise RuntimeError("Disable sent but unconfirmed; reconnect before further requests")
                raise RuntimeError("Communication is unsynchronized; reconnect before further requests")
            try:
                self._send_frame(com_type, address, data)
                response_address, response = self.acq_thread.response_queue.get(timeout=2)
                if response_address != address or len(response) != expected_size:
                    raise RuntimeError("Unexpected response; reconnect before further requests")
            except Exception as exc:
                self._desynchronized = True
                if isinstance(exc, queue.Empty):
                    raise TimeoutError("Response timed out; reconnect before further requests") from exc
                raise
            return response

    def read(self, address):
        register = self.registers.get(address)
        if register is None or not register["read"] or register["format"] is None:
            raise ValueError(f"Address {address} is not readable")
        fmt = register["format"]
        data = self._exchange(SFMComType.READ, address, expected_size=struct.calcsize(fmt))
        return struct.unpack(fmt, data)[0]

    def write(self, address, value=None):
        self.validate_write(address, value)
        fmt = self.registers[address]["format"]
        data = b"" if fmt is None else struct.pack(fmt, value)
        self._check_status(self._exchange(SFMComType.WRITE, address, data))
        return True

    @staticmethod
    def _check_status(response):
        status = struct.unpack("<b", response)[0]
        if status != 0:
            raise RuntimeError(f"Device returned error: {status}")

    def enable_streaming(self, address):
        self._check_status(self._exchange(SFMComType.ENABLE_STREAMING, address))

    def disable_streaming(self, address):
        self._check_status(self._exchange(SFMComType.DISABLE_STREAMING, address))

    # ----------------------------------------------------------------------------------

    def self_commissioning(self):
        self.set_start_measure_resistance()
        time.sleep(1)
        self.set_start_measure_ld()
        time.sleep(1)
        self.set_start_measure_lq()
        time.sleep(1)
        self.get_motor_param()
        self.set_foc_bandwidth(500)
        self.set_start_calibrate_abs_encoder()

    def get_motor_param(self):
        pole_pairs = self.get_pole_pairs()
        rs = self.get_rs()
        ld = self.get_ld()
        lq = self.get_lq()
        print(f'pole pairs:{pole_pairs}')
        print(f'Rs:{rs}')
        print(f'Ld:{ld}')
        print(f'Lq:{lq}')

    def set_foc_bandwidth(self,bw=100):
        rs = self.get_rs() / 2
        ld = self.get_ld() / 2
        lq = self.get_lq() / 2
        omega = 2 * np.pi * bw
        id_kp = ld * omega
        id_ki = rs * omega
        iq_kp = lq * omega
        iq_ki = rs * omega
        print(f'id: kp={id_kp} ki={id_ki}')
        print(f'iq: kp={iq_kp} ki={iq_ki}')
        self.set_id_kp(id_kp)
        self.set_id_ki(id_ki)
        self.set_id_deadband(0)
        self.set_iq_kp(iq_kp)
        self.set_iq_ki(iq_ki)
        self.set_iq_deadband(0)
