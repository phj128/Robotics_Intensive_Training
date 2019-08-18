# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: zss_debug.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='zss_debug.proto',
  package='',
  syntax='proto2',
  serialized_pb=_b('\n\x0fzss_debug.proto\"5\n\rRobots_Status\x12$\n\rrobots_status\x18\x01 \x03(\x0b\x32\r.Robot_Status\"X\n\x0cRobot_Status\x12\x10\n\x08robot_id\x18\x01 \x02(\x05\x12\x10\n\x08infrared\x18\x02 \x02(\x08\x12\x11\n\tflat_kick\x18\x03 \x02(\x08\x12\x11\n\tchip_kick\x18\x04 \x02(\x08\"@\n\x0eRobots_Command\x12\x1f\n\x07\x63ommand\x18\x01 \x03(\x0b\x32\x0e.Robot_Command\x12\r\n\x05\x64\x65lay\x18\x02 \x01(\x05\"\xbe\x01\n\rRobot_Command\x12\x10\n\x08robot_id\x18\x01 \x02(\x05\x12\x12\n\nvelocity_x\x18\x02 \x02(\x02\x12\x12\n\nvelocity_y\x18\x03 \x02(\x02\x12\x12\n\nvelocity_r\x18\x04 \x02(\x02\x12\x0c\n\x04kick\x18\x05 \x02(\x08\x12\r\n\x05power\x18\x06 \x02(\x02\x12\x15\n\rdribbler_spin\x18\x07 \x02(\x02\x12\x15\n\rcurrent_angle\x18\x08 \x01(\x02\x12\x14\n\x0ctarget_angle\x18\t \x01(\x02')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_ROBOTS_STATUS = _descriptor.Descriptor(
  name='Robots_Status',
  full_name='Robots_Status',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='robots_status', full_name='Robots_Status.robots_status', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=19,
  serialized_end=72,
)


_ROBOT_STATUS = _descriptor.Descriptor(
  name='Robot_Status',
  full_name='Robot_Status',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='robot_id', full_name='Robot_Status.robot_id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='infrared', full_name='Robot_Status.infrared', index=1,
      number=2, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='flat_kick', full_name='Robot_Status.flat_kick', index=2,
      number=3, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='chip_kick', full_name='Robot_Status.chip_kick', index=3,
      number=4, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=74,
  serialized_end=162,
)


_ROBOTS_COMMAND = _descriptor.Descriptor(
  name='Robots_Command',
  full_name='Robots_Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='command', full_name='Robots_Command.command', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='delay', full_name='Robots_Command.delay', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=164,
  serialized_end=228,
)


_ROBOT_COMMAND = _descriptor.Descriptor(
  name='Robot_Command',
  full_name='Robot_Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='robot_id', full_name='Robot_Command.robot_id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity_x', full_name='Robot_Command.velocity_x', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity_y', full_name='Robot_Command.velocity_y', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity_r', full_name='Robot_Command.velocity_r', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kick', full_name='Robot_Command.kick', index=4,
      number=5, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='power', full_name='Robot_Command.power', index=5,
      number=6, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dribbler_spin', full_name='Robot_Command.dribbler_spin', index=6,
      number=7, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_angle', full_name='Robot_Command.current_angle', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='target_angle', full_name='Robot_Command.target_angle', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=231,
  serialized_end=421,
)

_ROBOTS_STATUS.fields_by_name['robots_status'].message_type = _ROBOT_STATUS
_ROBOTS_COMMAND.fields_by_name['command'].message_type = _ROBOT_COMMAND
DESCRIPTOR.message_types_by_name['Robots_Status'] = _ROBOTS_STATUS
DESCRIPTOR.message_types_by_name['Robot_Status'] = _ROBOT_STATUS
DESCRIPTOR.message_types_by_name['Robots_Command'] = _ROBOTS_COMMAND
DESCRIPTOR.message_types_by_name['Robot_Command'] = _ROBOT_COMMAND

Robots_Status = _reflection.GeneratedProtocolMessageType('Robots_Status', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTS_STATUS,
  __module__ = 'zss_debug_pb2'
  # @@protoc_insertion_point(class_scope:Robots_Status)
  ))
_sym_db.RegisterMessage(Robots_Status)

Robot_Status = _reflection.GeneratedProtocolMessageType('Robot_Status', (_message.Message,), dict(
  DESCRIPTOR = _ROBOT_STATUS,
  __module__ = 'zss_debug_pb2'
  # @@protoc_insertion_point(class_scope:Robot_Status)
  ))
_sym_db.RegisterMessage(Robot_Status)

Robots_Command = _reflection.GeneratedProtocolMessageType('Robots_Command', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTS_COMMAND,
  __module__ = 'zss_debug_pb2'
  # @@protoc_insertion_point(class_scope:Robots_Command)
  ))
_sym_db.RegisterMessage(Robots_Command)

Robot_Command = _reflection.GeneratedProtocolMessageType('Robot_Command', (_message.Message,), dict(
  DESCRIPTOR = _ROBOT_COMMAND,
  __module__ = 'zss_debug_pb2'
  # @@protoc_insertion_point(class_scope:Robot_Command)
  ))
_sym_db.RegisterMessage(Robot_Command)


# @@protoc_insertion_point(module_scope)
