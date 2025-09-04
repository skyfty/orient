from typing import Dict
from typing import List
from typing import Text
from typing import Optional
import tempfile
import launch
import os


class ReplacePath(launch.Substitution):
  def __init__(self,
    path: Text,
    source_file: Text,
    name: launch.SomeSubstitutionsType) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
    self.__source_file = source_file
    self.__path = path
    self.__name = normalize_to_list_of_substitutions(name)

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    return self.__name

  @property
  def path(self) -> Text:
    """Getter for path."""
    return self.__path
  
  @property
  def source_file(self) -> Text:
    """Getter for source_file."""
    return self.__source_file

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    description_name = launch.utilities.perform_substitutions(context, self.name)
    return os.path.join(self.path, description_name, self.source_file)