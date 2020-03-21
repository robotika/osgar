# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/cylindergeom.proto

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
  name='ignition/msgs/cylindergeom.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n ignition/msgs/cylindergeom.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"U\n\x0c\x43ylinderGeom\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0e\n\x06radius\x18\x02 \x01(\x01\x12\x0e\n\x06length\x18\x03 \x01(\x01\x42\'\n\x11\x63om.ignition.msgsB\x12\x43ylinderGeomProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CYLINDERGEOM = _descriptor.Descriptor(
  name='CylinderGeom',
  full_name='ignition.msgs.CylinderGeom',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.CylinderGeom.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='radius', full_name='ignition.msgs.CylinderGeom.radius', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='length', full_name='ignition.msgs.CylinderGeom.length', index=2,
      number=3, type=1, cpp_type=5, label=1,
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
  serialized_start=79,
  serialized_end=164,
)

_CYLINDERGEOM.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['CylinderGeom'] = _CYLINDERGEOM

CylinderGeom = _reflection.GeneratedProtocolMessageType('CylinderGeom', (_message.Message,), dict(
  DESCRIPTOR = _CYLINDERGEOM,
  __module__ = 'ignition.msgs.cylindergeom_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.CylinderGeom)
  ))
_sym_db.RegisterMessage(CylinderGeom)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\022CylinderGeomProtos'))
# @@protoc_insertion_point(module_scope)
