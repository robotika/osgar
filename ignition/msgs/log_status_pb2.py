# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/log_status.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ignition.msgs import time_pb2 as ignition_dot_msgs_dot_time__pb2
from ignition.msgs import header_pb2 as ignition_dot_msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/log_status.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\x1eignition/msgs/log_status.proto\x12\rignition.msgs\x1a\x18ignition/msgs/time.proto\x1a\x1aignition/msgs/header.proto\"\xeb\x02\n\tLogStatus\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12%\n\x08sim_time\x18\x02 \x01(\x0b\x32\x13.ignition.msgs.Time\x12\x32\n\x08log_file\x18\x03 \x01(\x0b\x32 .ignition.msgs.LogStatus.LogFile\x1a\xdb\x01\n\x07LogFile\x12\x0b\n\x03uri\x18\x01 \x01(\t\x12\x11\n\tbase_path\x18\x02 \x01(\t\x12\x11\n\tfull_path\x18\x03 \x01(\t\x12\x0c\n\x04size\x18\x04 \x01(\x02\x12:\n\nsize_units\x18\x05 \x01(\x0e\x32&.ignition.msgs.LogStatus.LogFile.Units\x12\x18\n\x10record_resources\x18\x06 \x01(\x08\"9\n\x05Units\x12\t\n\x05\x42YTES\x10\x00\x12\x0b\n\x07K_BYTES\x10\x01\x12\x0b\n\x07M_BYTES\x10\x02\x12\x0b\n\x07G_BYTES\x10\x03\x42$\n\x11\x63om.ignition.msgsB\x0fLogStatusProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_time__pb2.DESCRIPTOR,ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_LOGSTATUS_LOGFILE_UNITS = _descriptor.EnumDescriptor(
  name='Units',
  full_name='ignition.msgs.LogStatus.LogFile.Units',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BYTES', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='K_BYTES', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='M_BYTES', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='G_BYTES', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=410,
  serialized_end=467,
)
_sym_db.RegisterEnumDescriptor(_LOGSTATUS_LOGFILE_UNITS)


_LOGSTATUS_LOGFILE = _descriptor.Descriptor(
  name='LogFile',
  full_name='ignition.msgs.LogStatus.LogFile',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uri', full_name='ignition.msgs.LogStatus.LogFile.uri', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='base_path', full_name='ignition.msgs.LogStatus.LogFile.base_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='full_path', full_name='ignition.msgs.LogStatus.LogFile.full_path', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='size', full_name='ignition.msgs.LogStatus.LogFile.size', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='size_units', full_name='ignition.msgs.LogStatus.LogFile.size_units', index=4,
      number=5, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='record_resources', full_name='ignition.msgs.LogStatus.LogFile.record_resources', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _LOGSTATUS_LOGFILE_UNITS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=248,
  serialized_end=467,
)

_LOGSTATUS = _descriptor.Descriptor(
  name='LogStatus',
  full_name='ignition.msgs.LogStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.LogStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sim_time', full_name='ignition.msgs.LogStatus.sim_time', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='log_file', full_name='ignition.msgs.LogStatus.log_file', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_LOGSTATUS_LOGFILE, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=104,
  serialized_end=467,
)

_LOGSTATUS_LOGFILE.fields_by_name['size_units'].enum_type = _LOGSTATUS_LOGFILE_UNITS
_LOGSTATUS_LOGFILE.containing_type = _LOGSTATUS
_LOGSTATUS_LOGFILE_UNITS.containing_type = _LOGSTATUS_LOGFILE
_LOGSTATUS.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_LOGSTATUS.fields_by_name['sim_time'].message_type = ignition_dot_msgs_dot_time__pb2._TIME
_LOGSTATUS.fields_by_name['log_file'].message_type = _LOGSTATUS_LOGFILE
DESCRIPTOR.message_types_by_name['LogStatus'] = _LOGSTATUS

LogStatus = _reflection.GeneratedProtocolMessageType('LogStatus', (_message.Message,), dict(

  LogFile = _reflection.GeneratedProtocolMessageType('LogFile', (_message.Message,), dict(
    DESCRIPTOR = _LOGSTATUS_LOGFILE,
    __module__ = 'ignition.msgs.log_status_pb2'
    # @@protoc_insertion_point(class_scope:ignition.msgs.LogStatus.LogFile)
    ))
  ,
  DESCRIPTOR = _LOGSTATUS,
  __module__ = 'ignition.msgs.log_status_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.LogStatus)
  ))
_sym_db.RegisterMessage(LogStatus)
_sym_db.RegisterMessage(LogStatus.LogFile)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\017LogStatusProtos'))
# @@protoc_insertion_point(module_scope)
