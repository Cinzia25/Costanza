# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vicon_receiver:msg/Position.idl
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


class Metaclass_Position(type):
    """Metaclass of message 'Position'."""

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
            module = import_type_support('vicon_receiver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vicon_receiver.msg.Position')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__position
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__position
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__position
            cls._TYPE_SUPPORT = module.type_support_msg__msg__position
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__position

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Position(metaclass=Metaclass_Position):
    """Message class 'Position'."""

    __slots__ = [
        '_x_trans',
        '_y_trans',
        '_z_trans',
        '_x_rot',
        '_y_rot',
        '_z_rot',
        '_w',
        '_x_rot_euler',
        '_y_rot_euler',
        '_z_rot_euler',
        '_segment_name',
        '_subject_name',
        '_frame_number',
        '_translation_type',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'x_trans': 'float',
        'y_trans': 'float',
        'z_trans': 'float',
        'x_rot': 'float',
        'y_rot': 'float',
        'z_rot': 'float',
        'w': 'float',
        'x_rot_euler': 'float',
        'y_rot_euler': 'float',
        'z_rot_euler': 'float',
        'segment_name': 'string',
        'subject_name': 'string',
        'frame_number': 'int32',
        'translation_type': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.x_trans = kwargs.get('x_trans', float())
        self.y_trans = kwargs.get('y_trans', float())
        self.z_trans = kwargs.get('z_trans', float())
        self.x_rot = kwargs.get('x_rot', float())
        self.y_rot = kwargs.get('y_rot', float())
        self.z_rot = kwargs.get('z_rot', float())
        self.w = kwargs.get('w', float())
        self.x_rot_euler = kwargs.get('x_rot_euler', float())
        self.y_rot_euler = kwargs.get('y_rot_euler', float())
        self.z_rot_euler = kwargs.get('z_rot_euler', float())
        self.segment_name = kwargs.get('segment_name', str())
        self.subject_name = kwargs.get('subject_name', str())
        self.frame_number = kwargs.get('frame_number', int())
        self.translation_type = kwargs.get('translation_type', str())

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
        if self.x_trans != other.x_trans:
            return False
        if self.y_trans != other.y_trans:
            return False
        if self.z_trans != other.z_trans:
            return False
        if self.x_rot != other.x_rot:
            return False
        if self.y_rot != other.y_rot:
            return False
        if self.z_rot != other.z_rot:
            return False
        if self.w != other.w:
            return False
        if self.x_rot_euler != other.x_rot_euler:
            return False
        if self.y_rot_euler != other.y_rot_euler:
            return False
        if self.z_rot_euler != other.z_rot_euler:
            return False
        if self.segment_name != other.segment_name:
            return False
        if self.subject_name != other.subject_name:
            return False
        if self.frame_number != other.frame_number:
            return False
        if self.translation_type != other.translation_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_trans(self):
        """Message field 'x_trans'."""
        return self._x_trans

    @x_trans.setter
    def x_trans(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_trans' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_trans' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_trans = value

    @builtins.property
    def y_trans(self):
        """Message field 'y_trans'."""
        return self._y_trans

    @y_trans.setter
    def y_trans(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_trans' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_trans' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_trans = value

    @builtins.property
    def z_trans(self):
        """Message field 'z_trans'."""
        return self._z_trans

    @z_trans.setter
    def z_trans(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z_trans' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_trans' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_trans = value

    @builtins.property
    def x_rot(self):
        """Message field 'x_rot'."""
        return self._x_rot

    @x_rot.setter
    def x_rot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_rot' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_rot' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_rot = value

    @builtins.property
    def y_rot(self):
        """Message field 'y_rot'."""
        return self._y_rot

    @y_rot.setter
    def y_rot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_rot' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_rot' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_rot = value

    @builtins.property
    def z_rot(self):
        """Message field 'z_rot'."""
        return self._z_rot

    @z_rot.setter
    def z_rot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z_rot' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_rot' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_rot = value

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'w' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._w = value

    @builtins.property
    def x_rot_euler(self):
        """Message field 'x_rot_euler'."""
        return self._x_rot_euler

    @x_rot_euler.setter
    def x_rot_euler(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_rot_euler' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_rot_euler' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_rot_euler = value

    @builtins.property
    def y_rot_euler(self):
        """Message field 'y_rot_euler'."""
        return self._y_rot_euler

    @y_rot_euler.setter
    def y_rot_euler(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_rot_euler' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_rot_euler' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_rot_euler = value

    @builtins.property
    def z_rot_euler(self):
        """Message field 'z_rot_euler'."""
        return self._z_rot_euler

    @z_rot_euler.setter
    def z_rot_euler(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z_rot_euler' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_rot_euler' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_rot_euler = value

    @builtins.property
    def segment_name(self):
        """Message field 'segment_name'."""
        return self._segment_name

    @segment_name.setter
    def segment_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'segment_name' field must be of type 'str'"
        self._segment_name = value

    @builtins.property
    def subject_name(self):
        """Message field 'subject_name'."""
        return self._subject_name

    @subject_name.setter
    def subject_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'subject_name' field must be of type 'str'"
        self._subject_name = value

    @builtins.property
    def frame_number(self):
        """Message field 'frame_number'."""
        return self._frame_number

    @frame_number.setter
    def frame_number(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'frame_number' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'frame_number' field must be an integer in [-2147483648, 2147483647]"
        self._frame_number = value

    @builtins.property
    def translation_type(self):
        """Message field 'translation_type'."""
        return self._translation_type

    @translation_type.setter
    def translation_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'translation_type' field must be of type 'str'"
        self._translation_type = value
