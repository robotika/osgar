# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/video_record.proto

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
  name='ignition/msgs/video_record.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n ignition/msgs/video_record.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"x\n\x0bVideoRecord\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\r\n\x05start\x18\x02 \x01(\x08\x12\x0c\n\x04stop\x18\x03 \x01(\x08\x12\x0e\n\x06\x66ormat\x18\x04 \x01(\t\x12\x15\n\rsave_filename\x18\x05 \x01(\tB&\n\x11\x63om.ignition.msgsB\x11VideoRecordProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_VIDEORECORD = _descriptor.Descriptor(
  name='VideoRecord',
  full_name='ignition.msgs.VideoRecord',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.VideoRecord.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start', full_name='ignition.msgs.VideoRecord.start', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop', full_name='ignition.msgs.VideoRecord.stop', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='format', full_name='ignition.msgs.VideoRecord.format', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='save_filename', full_name='ignition.msgs.VideoRecord.save_filename', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_end=199,
)

_VIDEORECORD.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
DESCRIPTOR.message_types_by_name['VideoRecord'] = _VIDEORECORD

VideoRecord = _reflection.GeneratedProtocolMessageType('VideoRecord', (_message.Message,), dict(
  DESCRIPTOR = _VIDEORECORD,
  __module__ = 'ignition.msgs.video_record_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.VideoRecord)
  ))
_sym_db.RegisterMessage(VideoRecord)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\021VideoRecordProtos'))
# @@protoc_insertion_point(module_scope)
