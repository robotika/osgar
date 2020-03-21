# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/visual_v.proto

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
from ignition.msgs import visual_pb2 as ignition_dot_msgs_dot_visual__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/visual_v.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1cignition/msgs/visual_v.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\x1a\x1aignition/msgs/visual.proto\"Y\n\x08Visual_V\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12&\n\x07visuals\x18\x02 \x03(\x0b\x32\x15.ignition.msgs.VisualB\"\n\x11\x63om.ignition.msgsB\rVisualVProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,ignition_dot_msgs_dot_visual__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_VISUAL_V = _descriptor.Descriptor(
  name='Visual_V',
  full_name='ignition.msgs.Visual_V',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Visual_V.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='visuals', full_name='ignition.msgs.Visual_V.visuals', index=1,
      number=2, type=11, cpp_type=10, label=3,
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=103,
  serialized_end=192,
)

_VISUAL_V.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_VISUAL_V.fields_by_name['visuals'].message_type = ignition_dot_msgs_dot_visual__pb2._VISUAL
DESCRIPTOR.message_types_by_name['Visual_V'] = _VISUAL_V

Visual_V = _reflection.GeneratedProtocolMessageType('Visual_V', (_message.Message,), dict(
  DESCRIPTOR = _VISUAL_V,
  __module__ = 'ignition.msgs.visual_v_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Visual_V)
  ))
_sym_db.RegisterMessage(Visual_V)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\rVisualVProtos'))
# @@protoc_insertion_point(module_scope)
