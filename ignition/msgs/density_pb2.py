# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/density.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import header_pb2 as ignition_dot_msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/density.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1bignition/msgs/density.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"A\n\x07\x44\x65nsity\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0f\n\x07\x64\x65nsity\x18\x02 \x01(\x01\x42\"\n\x11\x63om.ignition.msgsB\rDensityProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_DENSITY = _descriptor.Descriptor(
  name='Density',
  full_name='ignition.msgs.Density',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Density.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='density', full_name='ignition.msgs.Density.density', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=74,
  serialized_end=139,
)

_DENSITY.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['Density'] = _DENSITY

Density = _reflection.GeneratedProtocolMessageType('Density', (_message.Message,), dict(
  DESCRIPTOR = _DENSITY,
  __module__ = 'ignition.msgs.density_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Density)
  ))
_sym_db.RegisterMessage(Density)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\rDensityProtos'))
# @@protoc_insertion_point(module_scope)
