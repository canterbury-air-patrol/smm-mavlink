#!/usr/bin/env python3
"""
Connect a device that supports that mavlink to SMM as an asset
"""

import sys

from smm_client.connection import SMMConnection
from smm_client.types import SMMPoint

from pymavlink import mavutil, mavwp


class VehicleMav:
    """
    Mavlink connection to a vehicle
    """
    def __init__(self, connstr: str) -> None:
        self.conn = mavutil.mavlink_connection(connstr)
        self.conn.wait_heartbeat()
        self.wp = None

    @property
    def modes(self):
        """
        Get the flight modes this device supports
        """
        return self.conn.mode_mapping()

    def arm(self) -> None:
        """
        Arm this vehicle
        """
        print("Arming")
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

    def takeoff(self) -> None:
        """
        Set this vehicle to takeoff
        """
        if 'TAKEOFF' in self.modes:
            self.conn.mav.set_mode_send(self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.modes['TAKEOFF'])
            print("takeoff")

    def next_message(self):
        """
        Get the next message from this vehicle
        """
        while True:
            msg = self.conn.recv_match(blocking=True)
            if not msg:
                continue
            if msg.get_type() == "BAD_DATA":
                print("Bad data")
            else:
                if msg.get_type() in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                    print(self.wp.wp(msg.seq))
                    self.conn.mav.send(self.wp.wp(msg.seq))
                if msg.get_type() == "MISSION_ACK":
                    self.conn.mav.set_mode_send(self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.modes['AUTO'])
                    self.arm()
                return msg

    def set_continue(self):
        self.arm()

    def set_rtl(self) -> None:
        if 'RTL' in self.modes:
            self.conn.set_mode_send(self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.modes['RTL'])

    def set_circle(self) -> None:
        if 'CIRCLE' in self.modes:
            self.conn.set_mode_send(self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.modes['CIRCLE'])

    def set_goto(self, point) -> None:
        self.wp = mavwp.MAVWPLoader(self.conn.target_system)
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                1,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                int(point.lat * 10000000),
                int(point.lng * 10000000),
                int(20)
            )
        )
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                int(point.lat * 10000000),
                int(point.lng * 10000000),
                int(20)
            )
        )
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                2,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0.0,
                10.0,
                0.0,
                0.0,
                int(point.lat * 10000000),
                int(point.lng * 10000000),
                int(20)
            )
        )
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                3,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                0,
                1,
                0.0,
                0.0,
                50.0,
                0.0,
                int(point.lat * 10000000),
                int(point.lng * 10000000),
                20
            )
        )
        self.conn.waypoint_clear_all_send()
        print(self.wp.count())
        self.conn.waypoint_count_send(self.wp.count())


class SMM:
    def __init__(self, url, username, password, asset_name, autopilot: VehicleMav):
        self.mavlink = autopilot
        self.conn = SMMConnection(url, username, password)
        for asset in self.conn.get_assets():
            if asset.name == asset_name:
                self.asset = asset
        self.last_command_timestamp = None
        self.last_command = None

    def update_position(self, lat, lon, fix, alt, cog):
        cmd = self.asset.set_position(lat, lon, fix, alt, cog)
        if cmd is not None:
            self.process_cmd(cmd)

    def process_cmd(self, command):
        """
        Process an SMM Asset Command
        """
        if command.issued == self.last_command_timestamp:
            return
        self.last_command_timestamp = command.issued
        self.last_command = command.command
        if command.command == "Continue":
            # Set AP back to auto mode
            self.mavlink.set_continue()
        if command.command == "Circle":
            # Set AP to circle mode
            self.mavlink.set_circle()
        if command.command in ("Return to Launch", "Mission Complete"):
            # Set AP to RTL
            self.mavlink.set_rtl()
        if command.command == "Goto position":
            # Set AP to goto command.position
            self.mavlink.set_goto(command.position)
        print(command)


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print(f"Usage: {sys.argv[0]} MAVLINK SMMSERVER SMMUSER SMMPASSWORD SMMASSET")
        print("Where:")
        print("MAVLINK is /dev/ttyUSB0 or tcp:localhost:5760 or similar")
        print("SMMSERVER is the url for the Search Management Map Server")
        print("SMMUSER is the username for the Search Management Map Server")
        print("SMMPASSWORD is the password for the Search Management Map Server")
        print("SMMASSET is the name of the asset in the Search Management Map Server")
        sys.exit(-1)

    mavlink = VehicleMav(sys.argv[1])

    smm = SMM(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], mavlink)

    while True:
        msg = mavlink.next_message()
        if msg.get_type() == "GPS_RAW_INT":
            if msg.fix_type in (0, 1):
                continue
            smm.update_position(
                msg.lat / 10000000,
                msg.lon / 10000000,
                2 if msg.fix_type == 2 else 3,
                msg.alt / 1000,
                msg.cog / 100)
