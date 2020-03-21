# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/imagegeom.proto

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
  name='ignition/msgs/imagegeom.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1dignition/msgs/imagegeom.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"\x86\x01\n\tImageGeom\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0b\n\x03uri\x18\x02 \x01(\t\x12\r\n\x05scale\x18\x03 \x01(\x01\x12\x11\n\tthreshold\x18\x04 \x01(\x05\x12\x0e\n\x06height\x18\x05 \x01(\x01\x12\x13\n\x0bgranularity\x18\x06 \x01(\x05\x42$\n\x11\x63om.ignition.msgsB\x0fImageGeomProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_IMAGEGEOM = _descriptor.Descriptor(
  name='ImageGeom',
  full_name='ignition.msgs.ImageGeom',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.ImageGeom.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='uri', full_name='ignition.msgs.ImageGeom.uri', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scale', full_name='ignition.msgs.ImageGeom.scale', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='threshold', full_name='ignition.msgs.ImageGeom.threshold', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='ignition.msgs.ImageGeom.height', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='granularity', full_name='ignition.msgs.ImageGeom.granularity', index=5,
      number=6, type=5, cpp_type=1, label=1,
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=77,
  serialized_end=211,
)

_IMAGEGEOM.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['ImageGeom'] = _IMAGEGEOM

ImageGeom = _reflection.GeneratedProtocolMessageType('ImageGeom', (_message.Message,), dict(
  DESCRIPTOR = _IMAGEGEOM,
  __module__ = 'ignition.msgs.imagegeom_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.ImageGeom)
  ))
_sym_db.RegisterMessage(ImageGeom)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\017ImageGeomProtos'))
# @@protoc_insertion_point(module_scope)
