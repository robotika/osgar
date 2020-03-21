# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/air_pressure_sensor.proto

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
from ignition.msgs import sensor_noise_pb2 as ignition_dot_msgs_dot_sensor__noise__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ignition/msgs/air_pressure_sensor.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n\'ignition/msgs/air_pressure_sensor.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\x1a ignition/msgs/sensor_noise.proto\"\x8a\x01\n\x11\x41irPressureSensor\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x1a\n\x12reference_altitude\x18\x02 \x01(\x01\x12\x32\n\x0epressure_noise\x18\x03 \x01(\x0b\x32\x1a.ignition.msgs.SensorNoiseB,\n\x11\x63om.ignition.msgsB\x17\x41irPressureSensorProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,ignition_dot_msgs_dot_sensor__noise__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_AIRPRESSURESENSOR = _descriptor.Descriptor(
  name='AirPressureSensor',
  full_name='ignition.msgs.AirPressureSensor',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.AirPressureSensor.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reference_altitude', full_name='ignition.msgs.AirPressureSensor.reference_altitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pressure_noise', full_name='ignition.msgs.AirPressureSensor.pressure_noise', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=121,
  serialized_end=259,
)

_AIRPRESSURESENSOR.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_AIRPRESSURESENSOR.fields_by_name['pressure_noise'].message_type = ignition_dot_msgs_dot_sensor__noise__pb2._SENSORNOISE
DESCRIPTOR.message_types_by_name['AirPressureSensor'] = _AIRPRESSURESENSOR

AirPressureSensor = _reflection.GeneratedProtocolMessageType('AirPressureSensor', (_message.Message,), dict(
  DESCRIPTOR = _AIRPRESSURESENSOR,
  __module__ = 'ignition.msgs.air_pressure_sensor_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.AirPressureSensor)
  ))
_sym_db.RegisterMessage(AirPressureSensor)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\027AirPressureSensorProtos'))
# @@protoc_insertion_point(module_scope)
