"""Check default fixed_path, unique output routing and explicit mode compatibility."""
import itertools
import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, LoadComposableNodes

ROOT = Path(__file__).resolve().parents[1]


def value(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def active_nodes(actions, context):
    names = []
    for action in actions:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, Node):
            names.append(value(context, action._Node__node_name))
        elif isinstance(action, GroupAction):
            names.extend(active_nodes(action.get_sub_entities(), context))
        elif isinstance(action, LoadComposableNodes):
            for component in action._LoadComposableNodes__composable_node_descriptions:
                if component.condition() is None or component.condition().evaluate(context):
                    names.append(value(context, component.node_name))
    return names


class ControlAuthorityLaunchTest(unittest.TestCase):
    def test_default_and_explicit_modes(self):
        spec = importlib.util.spec_from_file_location('authority_launch', ROOT / 'launch/regulated_modules.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for mode, composition, gate in itertools.product(
                [None, 'fixed_path', 'autonomous', 'autonomous_fixed', 'remote'],
                [False, True], [False, True]):
            with self.subTest(mode=mode, composition=composition, gate=gate):
                context = LaunchContext()
                context.launch_configurations.update(
                    use_composition=str(composition),
                    use_collision_velocity_gate=str(gate).lower())
                if mode:
                    context.launch_configurations['operation_mode'] = mode
                description = module.generate_launch_description()
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                expected = mode or 'fixed_path'
                self.assertEqual(context.launch_configurations['operation_mode'], expected)
                self.assertEqual(context.launch_configurations['velocity_file_log_frequency'],
                                 '50.0' if expected == 'autonomous' else '100.0')
                names = active_nodes(description.entities, context)
                self.assertEqual(names.count('controlpub'), int(expected in ('autonomous', 'autonomous_fixed')))
                self.assertEqual(names.count('myagv_keyboard_control'), int(expected == 'remote'))
                self.assertEqual(names.count('regulated_navigator'), int(expected != 'remote'))


if __name__ == '__main__':
    unittest.main()
