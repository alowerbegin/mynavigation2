"""Resolve the real launch graph without creating ROS processes or sending commands."""
import importlib.util
import itertools
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    spec = importlib.util.spec_from_file_location('collision_module_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def value(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def active_nodes(actions, context):
    result = []
    for action in actions:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, Node):
            result.append(value(context, action._Node__node_name))
        elif isinstance(action, (GroupAction, TimerAction)):
            children = action.actions if isinstance(action, TimerAction) else action.get_sub_entities()
            result.extend(active_nodes(children, context))
        elif isinstance(action, LoadComposableNodes):
            for node in action._LoadComposableNodes__composable_node_descriptions:
                if node.condition() is None or node.condition().evaluate(context):
                    result.append(value(context, node.node_name))
    return result


class CollisionModuleLaunchTest(unittest.TestCase):
    def test_independent_status_and_velocity_gate(self):
        module = load(ROOT / 'launch/regulated_modules.launch.py')
        for mode, status, gate, translator, composition in itertools.product(
                ['fixed_path', 'autonomous_fixed'], [False, True], [False, True], [False, True], [False, True]):
            with self.subTest(mode=mode, status=status, gate=gate, translator=translator, composition=composition):
                context = LaunchContext()
                context.launch_configurations.update(
                    operation_mode=mode, use_collision_monitor=str(status).lower(),
                    use_collision_velocity_gate=str(gate).lower(),
                    use_obstacle_status=str(translator).lower(), use_composition=str(composition),
                    params_file=str(ROOT / 'params/regulated_modules.yaml'))
                description = module.generate_launch_description()
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                names = active_nodes(description.entities, context)
                self.assertEqual(names.count('car_op_zones'), int(status))
                self.assertEqual(names.count('car_op_status_sim'), int(status))
                self.assertEqual(names.count('obstacle_status_translator'), int(status and translator))
                self.assertEqual(names.count('collision_monitor'), int(gate))
                diagnostic = module.start_velocity_diagnostics(context)[0]
                params = evaluate_parameters(context, diagnostic._Node__parameters)[0]
                self.assertEqual(params['smoother_output_topic'], 'cmd_vel_collision_in' if gate else 'cmd_vel')

    def test_simulation_does_not_duplicate_status_chain(self):
        module = load(ROOT.parent / 'myworld_bringup/launch/entry.launch.py')
        for gate in [False, True]:
            context = LaunchContext()
            context.launch_configurations.update(use_collision_monitor='true', use_collision_velocity_gate=str(gate).lower())
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            def includes(actions):
                for action in actions:
                    if isinstance(action, IncludeLaunchDescription):
                        yield action
                    elif isinstance(action, (GroupAction, TimerAction)):
                        children = action.actions if isinstance(action, TimerAction) else action.get_sub_entities()
                        yield from includes(children)
            include = next(includes(description.entities))
            arguments = {k: value(context, v) for k, v in include.launch_arguments}
            self.assertEqual(arguments['use_collision_monitor'], 'false')
            self.assertEqual(arguments['use_obstacle_status'], 'false')
            self.assertEqual(arguments['use_collision_velocity_gate'], str(gate).lower())
            names = active_nodes(description.entities, context)
            self.assertEqual(names.count('obstacle_status_translator'), 1)


if __name__ == '__main__':
    unittest.main()
