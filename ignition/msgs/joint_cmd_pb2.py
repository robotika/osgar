# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/joint_cmd.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import double_pb2 as ignition_dot_msgs_dot_double__pb2
from ignition.msgs import pid_pb2 as ignition_dot_msgs_dot_pid__pb2
from ignition.msgs import header_pb2 as ignition_dot_msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/joint_cmd.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1dignition/msgs/joint_cmd.proto\x12\rignition.msgs\x1a\x1aignition/msgs/double.proto\x1a\x17ignition/msgs/pid.proto\x1a\x1aignition/msgs/header.proto\"\xea\x01\n\x08JointCmd\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0c\n\x04name\x18\x02 \x01(\t\x12\x0c\n\x04\x61xis\x18\x03 \x01(\x05\x12\x11\n\x05\x66orce\x18\x04 \x01(\x01\x42\x02\x18\x01\x12$\n\x08position\x18\x05 \x01(\x0b\x32\x12.ignition.msgs.PID\x12$\n\x08velocity\x18\x06 \x01(\x0b\x32\x12.ignition.msgs.PID\x12\r\n\x05reset\x18\x07 \x01(\x08\x12-\n\x0e\x66orce_optional\x18\x08 \x01(\x0b\x32\x15.ignition.msgs.DoubleB#\n\x11\x63om.ignition.msgsB\x0eJointCmdProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_double__pb2.DESCRIPTOR,ignition_dot_msgs_dot_pid__pb2.DESCRIPTOR,ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_JOINTCMD = _descriptor.Descriptor(
  name='JointCmd',
  full_name='ignition.msgs.JointCmd',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.JointCmd.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name', full_name='ignition.msgs.JointCmd.name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='axis', full_name='ignition.msgs.JointCmd.axis', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='force', full_name='ignition.msgs.JointCmd.force', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))),
    _descriptor.FieldDescriptor(
      name='position', full_name='ignition.msgs.JointCmd.position', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity', full_name='ignition.msgs.JointCmd.velocity', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reset', full_name='ignition.msgs.JointCmd.reset', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='force_optional', full_name='ignition.msgs.JointCmd.force_optional', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=130,
  serialized_end=364,
)

_JOINTCMD.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_JOINTCMD.fields_by_name['position'].message_type = ignition_dot_msgs_dot_pid__pb2._PID
_JOINTCMD.fields_by_name['velocity'].message_type = ignition_dot_msgs_dot_pid__pb2._PID
_JOINTCMD.fields_by_name['force_optional'].message_type = ignition_dot_msgs_dot_double__pb2._DOUBLE
DESCRIPTOR.message_types_by_name['JointCmd'] = _JOINTCMD

JointCmd = _reflection.GeneratedProtocolMessageType('JointCmd', (_message.Message,), dict(
  DESCRIPTOR = _JOINTCMD,
  __module__ = 'ignition.msgs.joint_cmd_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.JointCmd)
  ))
_sym_db.RegisterMessage(JointCmd)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\016JointCmdProtos'))
_JOINTCMD.fields_by_name['force'].has_options = True
_JOINTCMD.fields_by_name['force']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))
# @@protoc_insertion_point(module_scope)
