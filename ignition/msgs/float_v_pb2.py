# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/float_v.proto

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
  name='ignition/msgs/float_v.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1bignition/msgs/float_v.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\">\n\x07\x46loat_V\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x02\x42!\n\x11\x63om.ignition.msgsB\x0c\x46loatVProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_FLOAT_V = _descriptor.Descriptor(
  name='Float_V',
  full_name='ignition.msgs.Float_V',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Float_V.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data', full_name='ignition.msgs.Float_V.data', index=1,
      number=2, type=2, cpp_type=6, label=3,
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
  serialized_start=74,
  serialized_end=136,
)

_FLOAT_V.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['Float_V'] = _FLOAT_V

Float_V = _reflection.GeneratedProtocolMessageType('Float_V', (_message.Message,), dict(
  DESCRIPTOR = _FLOAT_V,
  __module__ = 'ignition.msgs.float_v_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Float_V)
  ))
_sym_db.RegisterMessage(Float_V)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\014FloatVProtos'))
# @@protoc_insertion_point(module_scope)
