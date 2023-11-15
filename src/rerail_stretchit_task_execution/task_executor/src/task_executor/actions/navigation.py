#Semantic Location has been removed from this code to make simpler for our use case
#!/usr/bin/env python3
import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from task_execution_msgs.msg import Waypoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from task_execution_msgs.srv import GetWaypoints
from actionlib_msgs.msg import GoalStatus
from math import sin, cos

class Navigation(AbstractStep):
    MOVE_ACTION_SERVER = "move_base"
    WAYPOINTS_SERVICE_NAME = "/database/waypoints"
        
    def init(self, name):
        self.name = name
        self._navigation_client = actionlib.SimpleActionClient(Navigation.MOVE_ACTION_SERVER, MoveBaseAction)
        self._get_waypoints_srv = rospy.ServiceProxy(Navigation.WAYPOINTS_SERVICE_NAME, GetWaypoints)
        self.action_goal = MoveBaseGoal()
        
        rospy.loginfo("Connecting to navigation...")
        self._navigation_client.wait_for_server()
        rospy.loginfo("...navigation connected")

        rospy.loginfo("Connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, location, update_belief=True):
        #Parse out the waypoints
        coords, semantic_location = self._parse_location(location)
        if coords is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, location))
            raise KeyError(self.name, "Unknown Format", location)

        rospy.logdebug("Action {}: Moving to location(s): {}".format(self.name, coords))

        status = GoalStatus.LOST
        for coord_num, coord in enumerate(coords):
            rospy.loginfo("Action {}: Going to {}/{}. Coordinate: {{ {} }}"
                          .format(self.name, coord_num + 1, len(coords), str(coord).replace("\n", ", ")))

            # Create and send the goal

            goal_pose = Pose()
            goal_pose.position.x = coord.x
            goal_pose.position.y = coord.y
            goal_pose.position.z = sin(coord.theta/2.0)
            goal_pose.orientation.w = cos(coord.theta/2.0)
            self.action_goal.target_pose.header.frame_id = coord.frame
            self.action_goal.target_pose.header.stamp = rospy.Time.now()
            self.action_goal.target_pose.pose = goal_pose
            
            self._navigation_client.send_goal(self.action_goal)
            self.notify_action_send_goal(Navigation.MOVE_ACTION_SERVER, self.action_goal)

            # Yield running while the move_client is executing
            while self._navigation_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status and stop executing if we didn't complete our goal
            status = self._navigation_client.get_state()
            self._navigation_client.wait_for_result()
            result = self._navigation_client.get_result()
            self.notify_action_recv_result(Navigation.MOVE_ACTION_SERVER, status, result)

            if status != GoalStatus.SUCCEEDED:
                break

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            # No update to task belief if move is successful since we dont have the semantic location for now
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=goal_pose,
                coord_num=coord_num,
                result=result,
                semantic_location=semantic_location
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=goal_pose,
                coord_num=coord_num,
                result=result,
                semantic_location=semantic_location
            )

    
    def stop(self):
        self._navigation_client.cancel_goal()
        self.notify_action_cancel(Navigation.MOVE_ACTION_SERVER)

    def _parse_location(self, location):
        coords = None
        semantic_location = None
        if isinstance(location, str):
            db_name, location = location.split('.', 1)
            if db_name == 'waypoints':
                coords = self._get_waypoints_srv(location).waypoints
                self.notify_service_called(Navigation.WAYPOINTS_SERVICE_NAME)
                semantic_location = location
            elif db_name == 'locations':
                # These are predefined tf frames
                coords = [Waypoint(frame=location)]
        elif isinstance(location, dict):
            coords = [Waypoint(**location),]
        elif isinstance(location, (list, tuple,)):
            coords = [Waypoint(**x) for x in location]

        return coords, semantic_location
    
    #removed the semantic location belief update