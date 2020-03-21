# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/version_range.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import version_pb2 as ignition_dot_msgs_dot_version__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/version_range.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n!ignition/msgs/version_range.proto\x12\rignition.msgs\x1a\x1bignition/msgs/version.proto\"X\n\x0cVersionRange\x12#\n\x03min\x18\x01 \x01(\x0b\x32\x16.ignition.msgs.Version\x12#\n\x03max\x18\x02 \x01(\x0b\x32\x16.ignition.msgs.VersionB!\n\x11\x63om.ignition.msgsB\x0cVersionRangeb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_version__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_VERSIONRANGE = _descriptor.Descriptor(
  name='VersionRange',
  full_name='ignition.msgs.VersionRange',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='min', full_name='ignition.msgs.VersionRange.min', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max', full_name='ignition.msgs.VersionRange.max', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  serialized_start=81,
  serialized_end=169,
)

_VERSIONRANGE.fields_by_name['min'].message_type = ignition_dot_msgs_dot_version__pb2._VERSION
_VERSIONRANGE.fields_by_name['max'].message_type = ignition_dot_msgs_dot_version__pb2._VERSION
DESCRIPTOR.message_types_by_name['VersionRange'] = _VERSIONRANGE

VersionRange = _reflection.GeneratedProtocolMessageType('VersionRange', (_message.Message,), dict(
  DESCRIPTOR = _VERSIONRANGE,
  __module__ = 'ignition.msgs.version_range_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.VersionRange)
  ))
_sym_db.RegisterMessage(VersionRange)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\014VersionRange'))
# @@protoc_insertion_point(module_scope)
