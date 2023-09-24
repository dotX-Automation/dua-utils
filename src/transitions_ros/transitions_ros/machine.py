"""
Implementation of a ROS 2-compatible transitions machine.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

September 11, 2022
"""

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

import transitions
from transitions_ros.state import *


class Machine(transitions.Machine):
    """Finite-state machine with ROS 2 capabilities."""

    # Use custom states
    state_cls = State

    def __init__(self,
                 node: NodeType,
                 states_table: list,
                 transitions_table: list,
                 initial_state: str) -> None:
        """
        Creates a new Machine, linking a ROS 2 node to it.

        :param node: ROS 2 node to link.
        :param states_table: List of states to use.
        :param transitions_table: List of transitions to use.
        :param initial_state: Initial state of the machine.
        """
        # Set internal attributes
        self._node = node

        # Initialize transitions machine
        super().__init__(
            states=states_table,
            transitions=transitions_table,
            initial=initial_state)

    def _create_state(self, *args, **kwargs) -> State:
        """Creates a new State object, overriding the default method."""
        return State(node=self._node, *args, **kwargs)

    def run(self) -> None:
        """Runs the active FSM."""
        while True:
            # Execute the routine of the current state and get the next trigger
            next_trigger = self.get_model_state(self).routine()

            # If there's no trigger, we're done here
            if len(next_trigger) == 0:
                break

            # Get to the next state
            self.trigger(next_trigger)
