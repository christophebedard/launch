# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Module for the ForLoop action."""

from typing import Dict
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Type


from ..action import Action
from ..actions.opaque_function import OpaqueFunction
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..logging import get_logger
from ..substitutions import LaunchConfiguration
from ..substitutions import LocalSubstitution


class ForLoop(Action):
    """
    Action that instantiates entities a given number of times based on a launch argument.

    The index value is made available through a substitution.

    A DeclareLaunchArgument must be created before this action to define the number of iterations
    in the for-loop, i.e., N iterations. The given entities are instantiated using the provided
    kwargs N times. For each loop iteration, a variable with the same name as the launch argument
    gets set to a unique value, going from 0 to N (exclusive). This variable can be used as an
    index in the entitity kwargs through a ForLoop.IndexSubstitution to differentiate the entities.

    Simple example:

    .. code-block:: python

        LaunchDescription([
            DeclareLaunchArgument('num', default_value='2'),
            ForLoop(
                'num',
                entities=[
                    (LogInfo, dict(
                        msg=['i=', ForLoop.IndexSubstitution('num')],
                    )),
                ],
            ),
        ])

    This would ouput the following log messages by default:

    .. code-block:: text

        i=0
        i=1

    If the launch argument was set to 5 (num:=5), then it would output:

    .. code-block:: text

        i=0
        i=1
        i=2
        i=3
        i=4
    """

    IndexSubstitution = LocalSubstitution

    def __init__(
        self,
        launch_argument_name: str,
        *,
        entities: List[Tuple[Type[LaunchDescriptionEntity], Dict]],
        **kwargs,
    ) -> None:
        """
        Create a ForLoop.

        :param launch_argument_name: the name of the launch argument that defines the length of the
            for-loop
        :param entities: the list of tuples of entity type and arguments, like
            (
                type of the entity to instantiate,
                entity constructor arguments
            )
        """
        super().__init__(**kwargs)
        self._launch_argument_name = launch_argument_name
        self._entities = entities
        self._logger = get_logger(__name__)

    @property
    def launch_argument_name(self) -> str:
        return self._launch_argument_name

    @property
    def entities(self) -> List[Tuple[Type[LaunchDescriptionEntity], Dict]]:
        return self._entities

    def describe(self) -> Text:
        return (
            type(self).__name__ +
            f"(launch_argument_name='{self._launch_argument_name}', entities={self._entities})"
        )

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        # Get the for-loop length and convert to int
        num = int(LaunchConfiguration(self._launch_argument_name).perform(context))
        self._logger.debug(f'for-loop length={num}')

        entities = []
        for i in range(num):
            entities.extend([
                # Push and pop locals to avoid having the index local leak
                OpaqueFunction(function=self._push_locals),
                # Set a local equal to i so that it can be used as a unique value by the entities
                OpaqueFunction(
                    function=self._set_index_local, args=(self._launch_argument_name, i)),
                # We can't just take in entity objects and include them here, otherwise they would
                # be executed multiple times, which is why we need separate instances and therefore
                # have to instantiate them here
                *[entity_t(**entity_kwargs) for entity_t, entity_kwargs in self._entities],
                OpaqueFunction(function=self._pop_locals),
            ])
        return entities

    def _push_locals(
        self,
        context: LaunchContext,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context._push_locals()
        return None

    def _pop_locals(
        self,
        context: LaunchContext,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context._pop_locals()
        return None

    def _set_index_local(
        self,
        context: LaunchContext,
        local_name: str,
        index: int,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        # Warn if index local already exists
        if local_name in context.locals:
            self._logger.warning(
                'local variable already exists: '
                f'{local_name}={context.locals.__getattr__(local_name)}')
        context.extend_locals({local_name: str(index)})
        return None
