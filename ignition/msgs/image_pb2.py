# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/image.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import header_pb2 as ignition_dot_msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/image.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x19ignition/msgs/image.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"\xa4\x01\n\x05Image\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\r\n\x05width\x18\x02 \x01(\r\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\x0c\n\x04step\x18\x04 \x01(\r\x12\x0c\n\x04\x64\x61ta\x18\x05 \x01(\x0c\x12\x39\n\x11pixel_format_type\x18\x06 \x01(\x0e\x32\x1e.ignition.msgs.PixelFormatType*\xbe\x02\n\x0fPixelFormatType\x12\x18\n\x14UNKNOWN_PIXEL_FORMAT\x10\x00\x12\n\n\x06L_INT8\x10\x01\x12\x0b\n\x07L_INT16\x10\x02\x12\x0c\n\x08RGB_INT8\x10\x03\x12\r\n\tRGBA_INT8\x10\x04\x12\r\n\tBGRA_INT8\x10\x05\x12\r\n\tRGB_INT16\x10\x06\x12\r\n\tRGB_INT32\x10\x07\x12\x0c\n\x08\x42GR_INT8\x10\x08\x12\r\n\tBGR_INT16\x10\t\x12\r\n\tBGR_INT32\x10\n\x12\r\n\tR_FLOAT16\x10\x0b\x12\x0f\n\x0bRGB_FLOAT16\x10\x0c\x12\r\n\tR_FLOAT32\x10\r\x12\x0f\n\x0bRGB_FLOAT32\x10\x0e\x12\x0f\n\x0b\x42\x41YER_RGGB8\x10\x0f\x12\x0f\n\x0b\x42\x41YER_BGGR8\x10\x10\x12\x0f\n\x0b\x42\x41YER_GBRG8\x10\x11\x12\x0f\n\x0b\x42\x41YER_GRBG8\x10\x12\x42 \n\x11\x63om.ignition.msgsB\x0bImageProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_PIXELFORMATTYPE = _descriptor.EnumDescriptor(
  name='PixelFormatType',
  full_name='ignition.msgs.PixelFormatType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_PIXEL_FORMAT', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L_INT8', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L_INT16', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGB_INT8', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGBA_INT8', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BGRA_INT8', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGB_INT16', index=6, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGB_INT32', index=7, number=7,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BGR_INT8', index=8, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BGR_INT16', index=9, number=9,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BGR_INT32', index=10, number=10,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='R_FLOAT16', index=11, number=11,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGB_FLOAT16', index=12, number=12,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='R_FLOAT32', index=13, number=13,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGB_FLOAT32', index=14, number=14,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BAYER_RGGB8', index=15, number=15,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BAYER_BGGR8', index=16, number=16,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BAYER_GBRG8', index=17, number=17,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BAYER_GRBG8', index=18, number=18,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=240,
  serialized_end=558,
)
_sym_db.RegisterEnumDescriptor(_PIXELFORMATTYPE)

PixelFormatType = enum_type_wrapper.EnumTypeWrapper(_PIXELFORMATTYPE)
UNKNOWN_PIXEL_FORMAT = 0
L_INT8 = 1
L_INT16 = 2
RGB_INT8 = 3
RGBA_INT8 = 4
BGRA_INT8 = 5
RGB_INT16 = 6
RGB_INT32 = 7
BGR_INT8 = 8
BGR_INT16 = 9
BGR_INT32 = 10
R_FLOAT16 = 11
RGB_FLOAT16 = 12
R_FLOAT32 = 13
RGB_FLOAT32 = 14
BAYER_RGGB8 = 15
BAYER_BGGR8 = 16
BAYER_GBRG8 = 17
BAYER_GRBG8 = 18



_IMAGE = _descriptor.Descriptor(
  name='Image',
  full_name='ignition.msgs.Image',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.Image.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='ignition.msgs.Image.width', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='ignition.msgs.Image.height', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='step', full_name='ignition.msgs.Image.step', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data', full_name='ignition.msgs.Image.data', index=4,
      number=5, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pixel_format_type', full_name='ignition.msgs.Image.pixel_format_type', index=5,
      number=6, type=14, cpp_type=8, label=1,
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
  serialized_start=73,
  serialized_end=237,
)

_IMAGE.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_IMAGE.fields_by_name['pixel_format_type'].enum_type = _PIXELFORMATTYPE
DESCRIPTOR.message_types_by_name['Image'] = _IMAGE
DESCRIPTOR.enum_types_by_name['PixelFormatType'] = _PIXELFORMATTYPE

Image = _reflection.GeneratedProtocolMessageType('Image', (_message.Message,), dict(
  DESCRIPTOR = _IMAGE,
  __module__ = 'ignition.msgs.image_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.Image)
  ))
_sym_db.RegisterMessage(Image)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\013ImageProtos'))
# @@protoc_insertion_point(module_scope)
