#__________________________________________________________________________80->|
# log.py
# Engineer: James W. Dunn
# This module logs flight telemetry
# Halt with a ctrl-c

import argparse
import time

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection  # noqa: F401

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection, tlog_name="TLog-manual.txt")

    def start(self):
        """
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
