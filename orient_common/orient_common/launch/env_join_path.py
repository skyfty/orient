# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the PathJoinSubstitution substitution."""

import os
from typing import Iterable
from typing import List
from typing import Text
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

class EnvPathJoinSubstitution(Substitution):

    def __init__(self, substitutions: Iterable[SomeSubstitutionsType]) -> None:
        """Create a PathJoinSubstitution substitution."""
        self.__substitutions = [
            normalize_to_list_of_substitutions(substitution)
            for substitution in substitutions
        ]
 

    @property
    def substitutions(self) -> List[List[Substitution]]:
        """Getter for variable_name."""
        return self.__substitutions

    def __repr__(self) -> Text:
        """Return a description of this substitution as a string."""
        path_components = [
            ' + '.join([s.describe() for s in component_substitutions])
            for component_substitutions in self.substitutions
        ]
        return f"EnvPathJoinSubstitution('{', '.join(path_components)}')"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitutions and join into a path."""
        path_components = [
            perform_substitutions(context, component_substitutions)
            for component_substitutions in self.substitutions
        ]
        return ":".join(path_components)

    def __truediv__(self, additional_path: SomeSubstitutionsType) -> 'EnvPathJoinSubstitution':
        """Join path substitutions using the / operator, mimicking pathlib.Path operation."""
        return EnvPathJoinSubstitution(
            self.substitutions + [normalize_to_list_of_substitutions(additional_path)])


