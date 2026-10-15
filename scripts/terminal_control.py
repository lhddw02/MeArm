"""Live degree and millimeter commands for the MuJoCo MeArm runner."""
import math

HELP = """Commands (angles in degrees; XYZ in millimeters):
  yaw 20                 rotate the base
  shoulder 65            upper-arm elevation (90 = vertical)
  forearm -20            forearm elevation (0 = horizontal)
  grip 15                open each jaw (0 = closed)
  joints 20 65 -20 15     set all four commands together
  xyz 120 30 120          move the tool to world X, Y, Z
  xyz 120 30 120 15       also set the jaw angle
  home                   return to 0, 90, 0, 0 degrees
  status                 show target angles and measured tool position
  help                   show these commands
  quit                   close the viewer
"""


def print_status(arm):
    angles = [math.degrees(float(value)) for value in arm.target]
    angles[1] += 90
    print('Target angles (deg): ' + ', '.join(
        f'{name}={value:.2f}' for name,value in zip(('yaw','shoulder','forearm','grip'),angles)))
    goal = arm.forward_kinematics()
    actual = arm.tcp_position_mm
    print('Target TCP (mm): ' + ', '.join(f'{name}={float(value):.2f}' for name,value in zip('XYZ',goal)))
    print('Current TCP (mm): ' + ', '.join(f'{name}={float(value):.2f}' for name,value in zip('XYZ',actual)))


def handle_command(arm, line):
    """Execute on the simulation thread. Return True only when asked to quit."""
    words = line.split()
    if not words:
        return False
    name = words[0].lower()
    arguments = words[1:]
    if name in ('quit','exit','q') and not arguments:
        return True
    if name in ('help','?') and not arguments:
        print(HELP)
        return False
    if name == 'status' and not arguments:
        print_status(arm)
        return False
    fields = {'yaw':'yaw_deg','shoulder':'shoulder_deg','forearm':'forearm_deg','grip':'grip_deg'}
    if name in fields and len(arguments)==1:
        arm.set_joint_targets(**{fields[name]:float(arguments[0])})
    elif name=='joints' and len(arguments)==4:
        arm.set_joint_targets(**dict(zip(fields.values(),map(float,arguments))))
    elif name=='xyz' and len(arguments) in (3,4):
        values=list(map(float,arguments))
        arm.move_to(*values[:3],grip_deg=values[3] if len(values)==4 else None)
    elif name=='home' and not arguments:
        arm.set_joint_targets(yaw_deg=0,shoulder_deg=90,forearm_deg=0,grip_deg=0)
    else:
        raise ValueError('Unknown command or wrong number of values. Type help for examples.')
    print_status(arm)
    return False
