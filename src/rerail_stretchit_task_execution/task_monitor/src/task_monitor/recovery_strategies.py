#!/usr/bin/env python3
# This file helps determine the recovery actions to take based on the request
# for assistance that was sent in the event of a failure

from __future__ import print_function, division

import copy
import pickle

import numpy as np

import rospy

import base64

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (RequestAssistanceResult, ExecuteGoal,
                                     BeliefKeys)
# from manipulation_actions.msg import StoreObjectResult, InHandLocalizeResult

from task_executor.actions import get_default_actions


# This class encapsulates the different strategies for recovering from different
# error situations

class RecoveryStrategies(object):
    """
    This class is responsible for determining the recovery task to execute when
    given the request for assistance goal and its corresponding context. The
    primary workhorse of this class is :meth:`get_strategy` which takes in a
    :class:`RequestAssistanceGoal`, and generates an :class:`ExecuteGoal`, a
    :class:`RequestAssistanceResult`, and a ``context`` on how to proceed when
    the recovery execution is complete.
    """

    # Just get all the BeliefKeys
    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]

    # Constant values that can dictate the behaviour of when to apply different
    # recovery behaviours
    MAX_PENULTIMATE_TASK_ABORTS = 7
    MAX_PRIMARY_TASK_ABORTS = 50

    def __init__(self, tasks_config):
        self._tasks_config = tasks_config
        self._actions = get_default_actions()

    def init(self):
        # Initialize the connections of all the actions
        self._actions.init()

    def get_strategy(self, assistance_goal):
        """
        Given an assistance goal, generate an ExecuteGoal that can be used for
        recovery and the corresponding manner of resumption. By default, we
        return a ``None`` ExecuteGoal and a
        :const:`RequestAssistanceResult.NONE`. The former implies that no goal
        should be executed, the latter that the task should be aborted in the
        event of a failure.

        In addition to task specific recoveries that are defined in the various
        ``if-elif-else`` conditions in this method, there are global recovery
        behaviours that apply to prevent infinite loops, for example:

            1. If the number of times the penultimate task in the hierarchy has \
                failed is > :const:`MAX_PENULTIMATE_TASK_ABORTS`, then the \
                recovery is aborted
            2. If the number of times the main task has aborted is > \
                :const:`MAX_PRIMARY_TASK_ABORTS`, then the recovery is aborted

        Args:
            assistance_goal (task_execution_msgs/RequestAssistanceGoal) :
                The request for assistance. The context attribute is unpickled

        Returns:
            (tuple):
                - execute_goal (``task_execution_msgs/ExecuteGoal``) a task \
                    goal to execute if any. If ``None``, assume there is no \
                    such goal
                - resume_hint (``RequestAssistanceResult.resume_hint``) a \
                    constant value indicating how execution should proceed
                - resume_context (dict) more fine grained control of the \
                    intended resume_hint
        """
        execute_goal = None
        resume_hint = RequestAssistanceResult.RESUME_NONE
        resume_context = { 'resume_hint': resume_hint }

        print("Looking for recovery strategy for {}".format(assistance_goal))

        # If our actions are not initialized, then recovery should fail because
        # this is an unknown scenario
        if not self._actions.initialized:
            rospy.logwarn("Recovery: cannot execute because actions are not initialized")
            return execute_goal, resume_hint, resume_context

        # Get the task beliefs. We don't expect it to fail
        _, beliefs = self._actions.get_beliefs(belief_keys=RecoveryStrategies.BELIEF_KEYS)

        # Get the number of times things have failed
        component_names, num_aborts = RecoveryStrategies.get_number_of_component_aborts(assistance_goal.context)
        
        # Then it's a giant lookup table. The first condition in the lookup
        # table is for test tasks. Should NEVER be used during the main task
        if (
            assistance_goal.component == 'loop_body_test'
            or assistance_goal.component == 'reposition_recovery_test'
        ):
            if assistance_goal.component == 'loop_body_test':
                rospy.loginfo("Recovery: simply continue")
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            elif assistance_goal.component == 'reposition_recovery_test':

                rospy.loginfo("Recovery: reposition the base")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "reposition back to an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=base64.b64encode(pickle.dumps(goal_params)).decode('ascii')
                )
        # Recovery for navigation
        elif assistance_goal.component == 'move':
            rospy.loginfo("Recovery: reposition, then retry move to goal pose")
            component_context = RecoveryStrategies.get_final_component_context(assistance_goal.context)
            self._actions.move_planar(angular_amount=np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(angular_amount=-1 * np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(linear_amount=-0.1)
            self._actions.wait(duration=0.5)
            print("component_context: {}".format(component_context))
            semantic_location_goal = component_context.get('semantic_location')
            if semantic_location_goal is not None:
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + semantic_location_goal,
                    "move_location": "waypoints." + semantic_location_goal
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=base64.b64encode(pickle.dumps(goal_params)).decode('ascii')
                )

            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
        
        # Return the recovery options
        rospy.loginfo("Recovery:\ngoal: {}\nresume_hint: {}\ncontext: {}".format(
            execute_goal if execute_goal is None else execute_goal.name,
            resume_hint,
            resume_context
        ))
        return execute_goal, resume_hint, resume_context

    @staticmethod
    def get_final_component_context(goal_context):
        """
        Get the context of the last component in the context chain
        """
        if len(goal_context.get('context', {})) > 0:
            return RecoveryStrategies.get_final_component_context(goal_context['context'])
        else:
            return goal_context

    @staticmethod
    def get_number_of_component_aborts(goal_context):
        """
        Given the hierarchy of tasks in the goal context, obtain a vector of the
        number of failures in each part of the component of the hierarchy. The
        first index maps to the highest level of the hierarcy and the last
        index maps to the lowest level of the hierarchy.

        Args:
            goal_context (dict) : the goal context

        Returns:
            (tuple):
                - component_names (list) a list of component names from highest \
                    in the task hierarchy to the lowest
                - num_aborts (list) a list of the number of times each \
                    component in component_names aborted
        """
        component_names = []
        num_aborts = []
        sub_context = goal_context

        while sub_context is not None and isinstance(sub_context, dict):
            component_names.append(sub_context.get('task') or sub_context.get('action'))
            num_aborts.append(sub_context.get('num_aborts'))
            sub_context = sub_context.get('context')

        # Return the lists
        return (component_names, num_aborts,)

    @staticmethod
    def create_continue_result_context(goal_context):
        """
        Given the context of a ``task_execution_msgs/RequestAssistanceGoal``
        return a dictionary for a ``task_execution_msgs/RequestAssistanceResult``
        context that indicates :const:`RequestAssistanceResult.RESUME_CONTINUE`

        Args:
            goal_context (dict) : the goal context

        Return:
            (dict) : the result context
        """
        if 'task' in goal_context:
            return {
                'task': goal_context['task'],
                'step_idx': goal_context['step_idx'],
                'resume_hint': RequestAssistanceResult.RESUME_CONTINUE,
                'context': RecoveryStrategies.create_continue_result_context(goal_context['context']),
            }
        else:
            return {}

    @staticmethod
    def set_task_hint_in_context(result_context, task_name, resume_hint):
        """
        Given a result context dictionary, mark the desired task name with the
        desired resume hint

        Args:
            result_context (dict) : the result context, possibly created by
                :meth:`create_continue_result_context`
            task_name (str) : the name of the task
            resume_hint (uint8) : A ``task_execution_msgs/RequestAssistanceResult`` \
                resume_hint constant for the task's resume hint

        Returns:
            (dict) : a result context dictionary with the task set to the \
                desired resume_hint. Note: we do not copy, so the incoming arg \
                might also get affected

        Raises:
            KeyError : if :data:`task_name` is not found in the context
        """

        # Error checking
        if 'task' not in result_context:
            raise KeyError("Expected a result context for tasks. Not found in {}!".format(result_context))

        # If this is not the task we want, then continue on to its context.
        # Otherwise, mark this task as the one we want to update and return
        if result_context['task'] == task_name:
            result_context['resume_hint'] = resume_hint
        else:
            result_context['context'] = RecoveryStrategies.set_task_hint_in_context(result_context['context'], task_name, resume_hint)

        return result_context

    @staticmethod
    def check_contradictory_beliefs(beliefs):
        """
        Given a set of beliefs, check if there is any contradiction.
        Currently only checking the contradiction between task and robot belief about its current location.

        Args:
            beliefs (dict) :

        Returns:
            (bool) : True if there is a contradiction, False otherwise
        """
        robot_beliefs = {}
        task_beliefs = {}
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                location = lower_belief_key.replace("task_at_", "")
                task_beliefs[location] = beliefs[belief_key]
            if "robot_at_" in lower_belief_key:
                location = lower_belief_key.replace("robot_at_", "")
                robot_beliefs[location] = beliefs[belief_key]
        for location in set(robot_beliefs.keys()) & set(task_beliefs.keys()):
            if robot_beliefs[location] != task_beliefs[location]:
                return True
        return False

    @staticmethod
    def get_last_goal_location(beliefs):
        """
        Given a set of beliefs, check the last location that the task says the
        robot was trying to get to.
        """
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                if beliefs[belief_key] == 1:
                    location = lower_belief_key.replace("task_at_", "")
                    return location
        return None