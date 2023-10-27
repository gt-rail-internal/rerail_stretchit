#!/usr/bin/env python3
import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from task_execution_msgs.msg import NavigationAction, NavigationGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from task_execution_msgs.srv import GetWaypoints

class Navigation(AbstractStep):
    MOVE_ACTION_SERVER = "/move_base"
    WAYPOINTS_SERVICE_NAME = "/database/waypoints"
    # SEMANTIC_LOCATIONS_SERVICE_NAME = "/database/semantic_locations"
    # BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]
        
    def init(self, name):
        print("Nav Serviceee! ", name)
        self.name = name
        self._navigation_client = actionlib.SimpleActionClient(Navigation.MOVE_ACTION_SERVER, MoveBaseAction)
        self._get_waypoints_srv = rospy.ServiceProxy(Navigation.WAYPOINTS_SERVICE_NAME, GetWaypoints)
        # self._get_semantic_locations_srv = rospy.ServiceProxy(
        #     MoveAction.SEMANTIC_LOCATIONS_SERVICE_NAME,
        #     GetSemanticLocations
        # )

        rospy.loginfo("Connecting to navigation...")
        self._navigation_client.wait_for_server()
        rospy.loginfo("...navigation connected")

        rospy.loginfo("Connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        # self._get_semantic_locations_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, location, update_belief=True):
        #Parse out the waypoints
        coords, semantic_location = self._parse_location(location)
        print(coords)
        print("Run Nav Service")
    
    def stop(self):
        print("Stop Nav Service")