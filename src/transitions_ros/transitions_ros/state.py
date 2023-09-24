"""
Implementation of the state of a ROS 2-compatible transitions machine.

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
from typing import TypeVar

NodeType = TypeVar('NodeType')
StateRoutineType = TypeVar('StateRoutineType')


class State(transitions.State):
    """Augmented machine state that also stores a ROS 2 node and a routine to execute."""

    def __init__(
            self,
            node: NodeType = None,
            routine: StateRoutineType = None,
            *args,
            **kwargs) -> None:
        """
        Creates a new State object.

        :param node: ROS 2 node to use.
        :param routine: Routine associated with this state.
        """
        # Set internal data for this implementation
        self._node = node
        self._routine = routine

        # Call State constructor
        super().__init__(*args, **kwargs)

    def routine(self) -> str:
        """
        Executes the routine associated with this state and returns the next trigger.

        :returns: Next trigger from the routine.
        """
        return self._routine(self._node)
