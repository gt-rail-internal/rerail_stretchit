#!/usr/bin/env python
# A service server that gives access to different data as needed
import rospy

from geometry_msgs.msg import PoseStamped
from task_execution_msgs.msg import Waypoint
from std_srvs.srv import Trigger, TriggerResponse
from task_execution_msgs.srv import (GetWaypoints, GetWaypointsResponse)


# The actual database node

class DatabaseServer(object):
    """
    Based on the ROS params that are loaded from a YAML file, this class
    provides a set of services that other nodes can use to query waypoints,
    arm trajectories, etc. by name
    """

    def __init__(self):
        # Provide a service to reload, then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload()

        # Start up the service servers for the different query types
        self._waypoints_service = rospy.Service(
            '~waypoints', GetWaypoints, self.get_waypoints
        )
   

    def start(self):
        # This is a no-op at the moment
        rospy.loginfo("Database node ready...")

    def reload(self, req=None):
        # Validate the data in each of the expected rosparams and populate the
        # database
        self.waypoints = self._validate_waypoints(rospy.get_param('~waypoints', {}))
        return TriggerResponse(success=True)

    def get_waypoints(self, req):
        resp = GetWaypointsResponse(waypoints=self.waypoints[req.name])
        return resp

    

    def _validate_waypoints(self, wp_defs):
        # Reload the waypoints
        waypoints = {}
        for name, wp_def in wp_defs.iteritems():
            waypoints[name] = [Waypoint(**x) for x in wp_def]

        return waypoints

    