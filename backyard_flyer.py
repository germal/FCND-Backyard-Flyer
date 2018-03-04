#__________________________________________________________________________80->|
# backyard_flyer.py
# Engineer: James W. Dunn
# This module flies a drone positionally in a square pattern (box mission).
# Targets can be adjusted at line 40

import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL    = 0
    ARMING    = 1
    TAKEOFF   = 2
    WAYPOINT  = 3
    LANDING   = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # Set initial state
        self.flight_state = States.MANUAL

        # Set targets
        self.target_altitude = 3.0  # meters
        self.NorthLegLength  = 10.0 # meters
        self.EastLegLength   = 10.0 # meters
        self.steps           = 5    # divisions along each leg, values 1 to 50
        
        # References
        self.globalhome = None
        self.localhome = None
        self.lastlocalheight = None

        # Register callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # See up_and_down.py, presented in the Udacity FCND classroom module...
        # "Project: Backyard Flyer, Lesson 11. A Simple Flight Plan"

    def local_position_callback(self):
        # This triggers when `MsgID.LOCAL_POSITION` is received 
        # and self.local_position contains new data
        if self.flight_state == States.WAYPOINT \
            and np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.35:
            if len(self.all_waypoints) > 0:
                self.waypoint_transition()
            else:
                if np.linalg.norm(self.local_velocity[0:2]) < 0.35:
                    self.landing_transition()
        # special case for takeoff
        elif self.flight_state == States.TAKEOFF:
            print("local height:", self.local_position[2])
            if self.localhome is None:
                self.localhome = self.local_position[2]
            if -self.local_position[2] > self.target_position[2] * 0.95:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

    def velocity_callback(self):
        # This triggers when `MsgID.LOCAL_VELOCITY` is received
        # and self.local_velocity contains new data
        if self.flight_state == States.LANDING:
            print("global delta height:", abs(self.global_position[2]-self.globalhome[2]))
            # check for global target elevation
            if abs(self.global_position[2]-self.globalhome[2]) < 0.25:
                print("local delta height:", abs(self.local_position[2]-self.localhome))
                # check for local target elevation
                if abs(self.local_position[2]-self.localhome) < 0.15:
                    # check for minimal altitude variation  
                    if self.lastlocalheight is not None and \
                        abs(self.local_position[2]-self.lastlocalheight) < 0.02 :
                        self.disarming_transition()
                        return
                    self.lastlocalheight = self.local_position[2]

    def state_callback(self):
        # This triggers when `MsgID.STATE` is received and self.armed
        # and self.guided contain new data
        if not self.in_mission:
            return
        if self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if ~self.armed & ~self.guided:
                self.manual_transition()
        elif self.flight_state == States.MANUAL:
            self.arming_transition()

    def calculate_box(self):
        """
        Returns waypoints to fly a box pattern.
        The 'box' is defined by the value self.NorthLegLength and self.EastLegLength
        and divided into increments using the value of self.steps which 
        can range from 1 through 50. 
        
        x -- + -- + -> x
        |              |   In the example to the left, the legs are divided into thirds.
        +              +   Five steps, each 2 meters apart, works well to hug the box.
        |              |   As the step value increases however, the path wobbles more.
        +              +   A step of one will drive the drone directly to the
        |              |   corner 'x' at the expense of overshooting the envelope.
        o <- + -- + -- x
        
        """
        print("Calculating target waypoints...")
        local_waypoints = []
        incrN = self.NorthLegLength / self.steps
        incrE = self.EastLegLength / self.steps
        # Northbound leg
        for i in range(self.steps):
            local_waypoints.append([(i+1)*incrN, 0.0, self.target_altitude])
        # Eastbound leg
        for i in range(self.steps):
            local_waypoints.append([self.NorthLegLength, (i+1)*incrE, self.target_altitude])
        # Southbound leg
        for i in range(self.steps):
            local_waypoints.append([(self.steps-1-i)*incrN, self.EastLegLength, self.target_altitude])
        # Westbound leg
        for i in range(self.steps):
            local_waypoints.append([0.0, (self.steps-1-i)*incrE, self.target_altitude])
        return local_waypoints

    def arming_transition(self):
        """
        If global positioning is available:
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        # per issue 96...check for uninitialized global positioning
        if self.global_position[0] == 0.0 and self.global_position[1] == 0.0: 
            print("no global position data, waiting...")
            return
        self.take_control()
        self.arm()
        print("setting home position to:", self.global_position[0], 
                               self.global_position[1],
                               self.global_position[2])
        self.set_home_position(self.global_position[0], 
                               self.global_position[1],
                               self.global_position[2])
        self.globalhome = [self.global_position[0], 
                           self.global_position[1],
                           self.global_position[2]]
        self.target_altitude = self.target_altitude + self.global_position[2]
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff and transition to altitude:", self.target_altitude)
        self.target_position[2] = self.target_altitude
        self.takeoff(self.target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """ 
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.target_position = self.all_waypoints.pop(0)
        print("waypoint transition to target:", self.target_position)
        self.cmd_position(self.target_position[0], 
                          self.target_position[1], 
                          self.target_position[2], 1.5)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """ 
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """ 
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

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
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
