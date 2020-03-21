# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/spherical_coordinates.proto

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
  name='ignition/msgs/spherical_coordinates.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n)ignition/msgs/spherical_coordinates.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"\xfc\x01\n\x14SphericalCoordinates\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12G\n\rsurface_model\x18\x02 \x01(\x0e\x32\x30.ignition.msgs.SphericalCoordinates.SurfaceModel\x12\x14\n\x0clatitude_deg\x18\x03 \x01(\x01\x12\x15\n\rlongitude_deg\x18\x04 \x01(\x01\x12\x11\n\televation\x18\x05 \x01(\x01\x12\x13\n\x0bheading_deg\x18\x06 \x01(\x01\"\x1f\n\x0cSurfaceModel\x12\x0f\n\x0b\x45\x41RTH_WGS84\x10\x00\x42/\n\x11\x63om.ignition.msgsB\x1aSphericalCoordinatesProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_SPHERICALCOORDINATES_SURFACEMODEL = _descriptor.EnumDescriptor(
  name='SurfaceModel',
  full_name='ignition.msgs.SphericalCoordinates.SurfaceModel',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='EARTH_WGS84', index=0, number=0,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=310,
  serialized_end=341,
)
_sym_db.RegisterEnumDescriptor(_SPHERICALCOORDINATES_SURFACEMODEL)


_SPHERICALCOORDINATES = _descriptor.Descriptor(
  name='SphericalCoordinates',
  full_name='ignition.msgs.SphericalCoordinates',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.SphericalCoordinates.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='surface_model', full_name='ignition.msgs.SphericalCoordinates.surface_model', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latitude_deg', full_name='ignition.msgs.SphericalCoordinates.latitude_deg', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_deg', full_name='ignition.msgs.SphericalCoordinates.longitude_deg', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='elevation', full_name='ignition.msgs.SphericalCoordinates.elevation', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='heading_deg', full_name='ignition.msgs.SphericalCoordinates.heading_deg', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SPHERICALCOORDINATES_SURFACEMODEL,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=89,
  serialized_end=341,
)

_SPHERICALCOORDINATES.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_SPHERICALCOORDINATES.fields_by_name['surface_model'].enum_type = _SPHERICALCOORDINATES_SURFACEMODEL
_SPHERICALCOORDINATES_SURFACEMODEL.containing_type = _SPHERICALCOORDINATES
DESCRIPTOR.message_types_by_name['SphericalCoordinates'] = _SPHERICALCOORDINATES

SphericalCoordinates = _reflection.GeneratedProtocolMessageType('SphericalCoordinates', (_message.Message,), dict(
  DESCRIPTOR = _SPHERICALCOORDINATES,
  __module__ = 'ignition.msgs.spherical_coordinates_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.SphericalCoordinates)
  ))
_sym_db.RegisterMessage(SphericalCoordinates)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\032SphericalCoordinatesProtos'))
# @@protoc_insertion_point(module_scope)
