#!/usr/bin/env python3
"""
Connect a device that supports mavlink to SMM as an asset
"""

import sys
from typing import Callable

from smm_client.connection import SMMConnection
from smm_client.missions import SMMMission
from smm_client.search import SMMSearch
from smm_client.types import SMMPoint

from pymavlink import mavutil, mavwp


class VehicleMav:
    """
    Mavlink connection to a vehicle
    """
    def __init__(self, connstr: str) -> None:
        self.conn = mavutil.mavlink_connection(connstr)
        self.conn.wait_heartbeat()
        self.search_id = None
        self.wp = None
        self.search_state = None
        self._events: dict[str, Callable] = {}

    def on(self, event: str, fn: Callable) -> None:
        """
        Register a callback for an event
        """
        self._events[event] = fn

    def _emit(self, event: str, *args) -> None:
        """
        Emit an event
        """
        if fn := self._events.pop(event, None):
            fn(*args)

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
            self.conn.mav.set_mode_send(
                self.conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.modes['TAKEOFF'])
            print("takeoff")

    MISSION_FIRST_ACTUAL_WAY_POINT_SEQUENCE = 3

    def _handle_mission_current(self, msg):
        """
        Handle a MISSION_CURRENT message
        """
        if msg.seq == self.MISSION_FIRST_ACTUAL_WAY_POINT_SEQUENCE \
                and msg.mission_state == mavutil.mavlink.MISSION_STATE_ACTIVE \
                and self.search_state == "Enroute":
            self._emit("search_begin", self.search_id)
            self.search_state = "Searching"
        if msg.mission_state == mavutil.mavlink.MISSION_STATE_COMPLETE \
                and self.search_state == "Searching":
            self._emit("search_complete", self.search_id)
            self.search_state = "Complete"
            self.search_id = None

    def _handle_mission_request(self, msg) -> None:
        """
        Handle a MISSION_REQUEST or MISSION_REQUEST_INT message
        """
        print(self.wp.wp(msg.seq))
        self.conn.mav.send(self.wp.wp(msg.seq))

    # pylint: disable=unused-argument
    def _handle_mission_ack(self, msg) -> None:
        """
        Handle a MISSION_ACK message
        """
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.modes['AUTO'])
        self.search_state = "Enroute"
        self.arm()

    def next_message(self):
        """
        Get the next message from this vehicle
        """
        handlers = {
            "MISSION_CURRENT": self._handle_mission_current,
            "MISSION_REQUEST": self._handle_mission_request,
            "MISSION_REQUEST_INT": self._handle_mission_request,
            "MISSION_ACK": self._handle_mission_ack,
        }
        while True:
            msg = self.conn.recv_match(blocking=True)
            if not msg or msg.get_type() == "BAD_DATA":
                continue
            mt = msg.get_type()
            if mt.startswith("MISSION") or mt == "STATUSTEXT":
                print(msg)
            if mt in handlers:
                handlers[mt](msg)
            return msg

    def set_rtl(self) -> None:
        """
        Set the auto-pilot to return to launch mode
        """
        if 'RTL' in self.modes:
            self.conn.mav.set_mode_send(
                self.conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.modes['RTL'])

    def set_circle(self) -> None:
        """
        Set the auto-pilot to circle/hold at the current location
        """
        if 'CIRCLE' in self.modes:
            self.conn.mav.set_mode_send(
                self.conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.modes['CIRCLE'])

    def set_goto(self, point: SMMPoint) -> None:
        """
        Goto a speific point
        """
        self.search_id = None
        self.wp = mavwp.MAVWPLoader(self.conn.target_system)
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
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
                1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                1,
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

    def load_search(self,
                    coords: list[SMMPoint],
                    search_id: int,
                    on_search_begin: Callable | None = None,
                    on_search_complete: Callable | None = None) -> None:
        """
        Load a search into the autopilot
        """
        self.search_id = search_id
        if on_search_begin:
            self.on("search_begin", on_search_begin)
        if on_search_complete:
            self.on("search_complete", on_search_complete)
        self.search_state = "Loading"
        point = coords[0]
        self.wp = mavwp.MAVWPLoader(self.conn.target_system)
        self.wp.add(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.conn.target_system,
                self.conn.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
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
                1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                1,
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
        seq = 2
        for point in coords:
            self.wp.add(
                mavutil.mavlink.MAVLink_mission_item_int_message(
                    self.conn.target_system,
                    self.conn.target_component,
                    seq,
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
            seq = seq + 1
        self.conn.waypoint_clear_all_send()
        print(self.wp.count())
        self.conn.waypoint_count_send(self.wp.count())


class SMM:
    # pylint: disable=R0902
    """
    Class to interact with SMM as an asset
    """
    def __init__(self, url, username, password, asset_name, autopilot: VehicleMav) -> None:
        # pylint: disable=R0913,R0917
        self.mavlink = autopilot
        self.conn = SMMConnection(url, username, password)
        for asset in self.conn.get_assets():
            if asset.name == asset_name:
                self.asset = asset
        self.last_command_timestamp = None
        self.last_command = None
        self.current_search: SMMSearch | None = None
        self.mission: SMMMission | None = None
        self.mission_asset_statuses = {}
        for mission_status in self.conn.get_mission_asset_status_values():
            self.mission_asset_statuses[mission_status.name] = mission_status

    def update_mission(self):
        """
        Update which mission this asset is in
        """
        self.mission = SMMMission.get_mission_for_asset(self.asset)
        return self.mission is not None

    def set_mission_status(self, status_name: str, status_text: str = ""):
        """
        Set the status of this asset in the mission
        """
        if status_name in self.mission_asset_statuses and self.update_mission():
            self.mission.set_asset_status(
                self.asset,
                self.mission_asset_statuses[status_name],
                status_text)

    def search_begin(self, search_id: int):
        """
        Callback for when a search begins
        """
        if self.current_search is not None and self.current_search.id == search_id:
            self.set_mission_status("Searching")

    def search_complete(self, search_id: int):
        """
        Callback for when a search is complete
        """
        if self.current_search is not None and self.current_search.id == search_id:
            self.current_search.finished(self.asset)
            self.set_mission_status("Search Complete")
            self.current_search = None
            self.mavlink.set_rtl()

    def load_search(self, lat, lon):
        """
        Load a search into the flight controller
        """
        if self.current_search is None:
            search = self.asset.get_next_search(lat, lon)
            if search is not None and search.begin(self.asset):
                self.current_search = search
            else:
                return
        if self.mavlink.search_id != self.current_search.id:
            print(self.current_search.get_data())
            self.mavlink.load_search(
                self.current_search.get_data().coords,
                self.current_search.id,
                self.search_begin,
                self.search_complete)
            self.set_mission_status("Enroute")

    def update_position(self, lat, lon, fix, alt, cog) -> None:
        # pylint: disable=R0913,R0917
        """
        Update the position of this asset
        """
        cmd = self.asset.set_position(lat, lon, fix, alt, cog)
        if cmd is not None:
            self.process_cmd(cmd)
        if self.last_command == "Continue":
            self.load_search(lat, lon)

    def process_cmd(self, command):
        """
        Process an SMM Asset Command
        """
        if command.issued == self.last_command_timestamp:
            return
        self.last_command_timestamp = command.issued
        self.last_command = command.command
        if command.command == "Circle":
            # Set AP to circle mode
            self.mavlink.set_circle()
        if command.command in ("Return to Launch", "Mission Complete"):
            # Set AP to RTL
            self.set_mission_status("Returning to Base")
            self.mavlink.set_rtl()
        if command.command == "Goto position":
            # Set AP to goto command.position
            self.mavlink.set_goto(command.position)
            self.set_mission_status("Investigating")
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
        mavmsg = mavlink.next_message()
        if mavmsg.get_type() == "GPS_RAW_INT":
            if mavmsg.fix_type in (0, 1):
                continue
            smm.update_position(
                mavmsg.lat / 10000000,
                mavmsg.lon / 10000000,
                2 if mavmsg.fix_type == 2 else 3,
                mavmsg.alt / 1000,
                mavmsg.cog / 100)
