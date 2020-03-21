# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/request.proto

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
  name='ignition/msgs/request.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1bignition/msgs/request.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"m\n\x07Request\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\n\n\x02id\x18\x02 \x01(\x05\x12\x0f\n\x07request\x18\x03 \x01(\t\x12\x0c\n\x04\x64\x61ta\x18\x04 \x01(\t\x12\x10\n\x08\x64\x62l_data\x18\x05 \x01(\x01\x42\"\n\x11\x63om.ignition.msgsB\rRequestProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_REQUEST = _descriptor.Descriptor(
  name='Request',
  full_name='ignition.msgs.Request',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Request.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='id', full_name='ignition.msgs.Request.id', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='request', full_name='ignition.msgs.Request.request', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data', full_name='ignition.msgs.Request.data', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dbl_data', full_name='ignition.msgs.Request.dbl_data', index=4,
      number=5, type=1, cpp_type=5, label=1,
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
  serialized_end=183,
)

_REQUEST.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['Request'] = _REQUEST

Request = _reflection.GeneratedProtocolMessageType('Request', (_message.Message,), dict(
  DESCRIPTOR = _REQUEST,
  __module__ = 'ignition.msgs.request_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Request)
  ))
_sym_db.RegisterMessage(Request)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\rRequestProtos'))
# @@protoc_insertion_point(module_scope)
