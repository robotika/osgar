# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/param_v.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import param_pb2 as ignition_dot_msgs_dot_param__pb2
from ignition.msgs import header_pb2 as ignition_dot_msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/param_v.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1bignition/msgs/param_v.proto\x12\rignition.msgs\x1a\x19ignition/msgs/param.proto\x1a\x1aignition/msgs/header.proto\"U\n\x07Param_V\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12#\n\x05param\x18\x02 \x03(\x0b\x32\x14.ignition.msgs.ParamB \n\x11\x63om.ignition.msgsB\x0b\x45mptyProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_param__pb2.DESCRIPTOR,ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_PARAM_V = _descriptor.Descriptor(
  name='Param_V',
  full_name='ignition.msgs.Param_V',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Param_V.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='param', full_name='ignition.msgs.Param_V.param', index=1,
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
  serialized_start=101,
  serialized_end=186,
)

_PARAM_V.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_PARAM_V.fields_by_name['param'].message_type = ignition_dot_msgs_dot_param__pb2._PARAM
DESCRIPTOR.message_types_by_name['Param_V'] = _PARAM_V

Param_V = _reflection.GeneratedProtocolMessageType('Param_V', (_message.Message,), dict(
  DESCRIPTOR = _PARAM_V,
  __module__ = 'ignition.msgs.param_v_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Param_V)
  ))
_sym_db.RegisterMessage(Param_V)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\013EmptyProtos'))
# @@protoc_insertion_point(module_scope)
