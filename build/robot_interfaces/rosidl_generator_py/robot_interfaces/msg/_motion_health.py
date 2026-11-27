# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/MotionHealth.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotionHealth(type):
    """Metaclass of message 'MotionHealth'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_interfaces.msg.MotionHealth')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motion_health
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motion_health
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motion_health
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motion_health
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motion_health

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotionHealth(metaclass=Metaclass_MotionHealth):
    """Message class 'MotionHealth'."""

    __slots__ = [
        '_state',
        '_reason',
        '_v',
        '_w',
        '_sigma_xy',
        '_sigma_theta',
        '_drift_deg_per_m',
        '_visual_known',
        '_visual_moving',
        '_since_s',
        '_imu_known',
        '_pitch_deg',
        '_roll_deg',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'state': 'string',
        'reason': 'string',
        'v': 'double',
        'w': 'double',
        'sigma_xy': 'double',
        'sigma_theta': 'double',
        'drift_deg_per_m': 'double',
        'visual_known': 'boolean',
        'visual_moving': 'boolean',
        'since_s': 'double',
        'imu_known': 'boolean',
        'pitch_deg': 'double',
        'roll_deg': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = kwargs.get('state', str())
        self.reason = kwargs.get('reason', str())
        self.v = kwargs.get('v', float())
        self.w = kwargs.get('w', float())
        self.sigma_xy = kwargs.get('sigma_xy', float())
        self.sigma_theta = kwargs.get('sigma_theta', float())
        self.drift_deg_per_m = kwargs.get('drift_deg_per_m', float())
        self.visual_known = kwargs.get('visual_known', bool())
        self.visual_moving = kwargs.get('visual_moving', bool())
        self.since_s = kwargs.get('since_s', float())
        self.imu_known = kwargs.get('imu_known', bool())
        self.pitch_deg = kwargs.get('pitch_deg', float())
        self.roll_deg = kwargs.get('roll_deg', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.state != other.state:
            return False
        if self.reason != other.reason:
            return False
        if self.v != other.v:
            return False
        if self.w != other.w:
            return False
        if self.sigma_xy != other.sigma_xy:
            return False
        if self.sigma_theta != other.sigma_theta:
            return False
        if self.drift_deg_per_m != other.drift_deg_per_m:
            return False
        if self.visual_known != other.visual_known:
            return False
        if self.visual_moving != other.visual_moving:
            return False
        if self.since_s != other.since_s:
            return False
        if self.imu_known != other.imu_known:
            return False
        if self.pitch_deg != other.pitch_deg:
            return False
        if self.roll_deg != other.roll_deg:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value

    @builtins.property
    def v(self):
        """Message field 'v'."""
        return self._v

    @v.setter
    def v(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'v' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v = value

    @builtins.property
    def w(self):
        """Message field 'w'."""
        return self._w

    @w.setter
    def w(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'w' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'w' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._w = value

    @builtins.property
    def sigma_xy(self):
        """Message field 'sigma_xy'."""
        return self._sigma_xy

    @sigma_xy.setter
    def sigma_xy(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'sigma_xy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'sigma_xy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._sigma_xy = value

    @builtins.property
    def sigma_theta(self):
        """Message field 'sigma_theta'."""
        return self._sigma_theta

    @sigma_theta.setter
    def sigma_theta(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'sigma_theta' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'sigma_theta' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._sigma_theta = value

    @builtins.property
    def drift_deg_per_m(self):
        """Message field 'drift_deg_per_m'."""
        return self._drift_deg_per_m

    @drift_deg_per_m.setter
    def drift_deg_per_m(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'drift_deg_per_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'drift_deg_per_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._drift_deg_per_m = value

    @builtins.property
    def visual_known(self):
        """Message field 'visual_known'."""
        return self._visual_known

    @visual_known.setter
    def visual_known(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'visual_known' field must be of type 'bool'"
        self._visual_known = value

    @builtins.property
    def visual_moving(self):
        """Message field 'visual_moving'."""
        return self._visual_moving

    @visual_moving.setter
    def visual_moving(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'visual_moving' field must be of type 'bool'"
        self._visual_moving = value

    @builtins.property
    def since_s(self):
        """Message field 'since_s'."""
        return self._since_s

    @since_s.setter
    def since_s(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'since_s' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'since_s' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._since_s = value

    @builtins.property
    def imu_known(self):
        """Message field 'imu_known'."""
        return self._imu_known

    @imu_known.setter
    def imu_known(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'imu_known' field must be of type 'bool'"
        self._imu_known = value

    @builtins.property
    def pitch_deg(self):
        """Message field 'pitch_deg'."""
        return self._pitch_deg

    @pitch_deg.setter
    def pitch_deg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_deg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_deg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_deg = value

    @builtins.property
    def roll_deg(self):
        """Message field 'roll_deg'."""
        return self._roll_deg

    @roll_deg.setter
    def roll_deg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll_deg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll_deg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._roll_deg = value
