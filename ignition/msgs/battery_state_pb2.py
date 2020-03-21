# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ignition/msgs/battery_state.proto

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
  name='ignition/msgs/battery_state.proto',
  package='ignition.msgs',
  syntax='proto3',
  serialized_pb=_b('\n!ignition/msgs/battery_state.proto\x12\rignition.msgs\x1a\x1aignition/msgs/header.proto\"\xb6\x02\n\x0c\x42\x61tteryState\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.ignition.msgs.Header\x12\x0f\n\x07voltage\x18\x02 \x01(\x01\x12\x0f\n\x07\x63urrent\x18\x03 \x01(\x01\x12\x0e\n\x06\x63harge\x18\x04 \x01(\x01\x12\x10\n\x08\x63\x61pacity\x18\x05 \x01(\x01\x12\x12\n\npercentage\x18\x06 \x01(\x01\x12J\n\x13power_supply_status\x18\x07 \x01(\x0e\x32-.ignition.msgs.BatteryState.PowerSupplyStatus\"[\n\x11PowerSupplyStatus\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x0c\n\x08\x43HARGING\x10\x01\x12\x0f\n\x0b\x44ISCHARGING\x10\x02\x12\x10\n\x0cNOT_CHARGING\x10\x03\x12\x08\n\x04\x46ULL\x10\x04\x42\'\n\x11\x63om.ignition.msgsB\x12\x42\x61tteryStateProtosb\x06proto3')
  ,
  dependencies=[ignition_dot_msgs_dot_header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_BATTERYSTATE_POWERSUPPLYSTATUS = _descriptor.EnumDescriptor(
  name='PowerSupplyStatus',
  full_name='ignition.msgs.BatteryState.PowerSupplyStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHARGING', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DISCHARGING', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NOT_CHARGING', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FULL', index=4, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=300,
  serialized_end=391,
)
_sym_db.RegisterEnumDescriptor(_BATTERYSTATE_POWERSUPPLYSTATUS)


_BATTERYSTATE = _descriptor.Descriptor(
  name='BatteryState',
  full_name='ignition.msgs.BatteryState',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ignition.msgs.BatteryState.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='voltage', full_name='ignition.msgs.BatteryState.voltage', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current', full_name='ignition.msgs.BatteryState.current', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='charge', full_name='ignition.msgs.BatteryState.charge', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='capacity', full_name='ignition.msgs.BatteryState.capacity', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='percentage', full_name='ignition.msgs.BatteryState.percentage', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='power_supply_status', full_name='ignition.msgs.BatteryState.power_supply_status', index=6,
      number=7, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _BATTERYSTATE_POWERSUPPLYSTATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=81,
  serialized_end=391,
)

_BATTERYSTATE.fields_by_name['header'].message_type = ignition_dot_msgs_dot_header__pb2._HEADER
_BATTERYSTATE.fields_by_name['power_supply_status'].enum_type = _BATTERYSTATE_POWERSUPPLYSTATUS
_BATTERYSTATE_POWERSUPPLYSTATUS.containing_type = _BATTERYSTATE
DESCRIPTOR.message_types_by_name['BatteryState'] = _BATTERYSTATE

BatteryState = _reflection.GeneratedProtocolMessageType('BatteryState', (_message.Message,), dict(
  DESCRIPTOR = _BATTERYSTATE,
  __module__ = 'ignition.msgs.battery_state_pb2'
  # @@protoc_insertion_point(class_scope:ignition.msgs.BatteryState)
  ))
_sym_db.RegisterMessage(BatteryState)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\021com.ignition.msgsB\022BatteryStateProtos'))
# @@protoc_insertion_point(module_scope)
