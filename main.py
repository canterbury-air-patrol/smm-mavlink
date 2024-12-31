#!/usr/bin/env python3
"""
Connect a device that supports that mavlink to SMM as an asset
"""

import sys

from smm_client.connection import SMMConnection

from pymavlink import mavutil


def process_cmd(command):
    """
    Process an SMM Asset Command
    """
    if command is None:
        return
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

    mavlink = mavutil.mavlink_connection(sys.argv[1])

    smm = SMMConnection(sys.argv[2], sys.argv[3], sys.argv[4])
    SMM_ASSET = None
    for asset in smm.get_assets():
        if asset.name == sys.argv[5]:
            SMM_ASSET = asset
    if SMM_ASSET is None:
        print(f"No such asset {sys.argv[5]}")
        sys.exit(-1)

    while True:
        msg = mavlink.recv_match(blocking=True)
        if not msg:
            continue
        if msg.get_type() == "BAD_DATA":
            print("Bad data")
        else:
            if msg.get_type() == "GPS_RAW_INT":
                if msg.fix_type in (0, 1):
                    continue
                print(msg)
                print(f"{msg.lat / 10000000} {msg.lon / 10000000}")
                cmd = SMM_ASSET.set_position(
                    msg.lat / 10000000,
                    msg.lon / 10000000,
                    2 if msg.fix_type == 2 else 3,
                    msg.alt / 1000,
                    msg.cog / 100)
                process_cmd(cmd)
