# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rosdemo_v4/MovJRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MovJRequest(genpy.Message):
  _md5sum = "87e876bc8a2d6739c81f81ad324bafb5"
  _type = "rosdemo_v4/MovJRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool mode
float64 a
float64 b
float64 c
float64 d
float64 e
float64 f
string[] paramValue
"""
  __slots__ = ['mode','a','b','c','d','e','f','paramValue']
  _slot_types = ['bool','float64','float64','float64','float64','float64','float64','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mode,a,b,c,d,e,f,paramValue

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MovJRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.mode is None:
        self.mode = False
      if self.a is None:
        self.a = 0.
      if self.b is None:
        self.b = 0.
      if self.c is None:
        self.c = 0.
      if self.d is None:
        self.d = 0.
      if self.e is None:
        self.e = 0.
      if self.f is None:
        self.f = 0.
      if self.paramValue is None:
        self.paramValue = []
    else:
      self.mode = False
      self.a = 0.
      self.b = 0.
      self.c = 0.
      self.d = 0.
      self.e = 0.
      self.f = 0.
      self.paramValue = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_B6d().pack(_x.mode, _x.a, _x.b, _x.c, _x.d, _x.e, _x.f))
      length = len(self.paramValue)
      buff.write(_struct_I.pack(length))
      for val1 in self.paramValue:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 49
      (_x.mode, _x.a, _x.b, _x.c, _x.d, _x.e, _x.f,) = _get_struct_B6d().unpack(str[start:end])
      self.mode = bool(self.mode)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.paramValue = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.paramValue.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_B6d().pack(_x.mode, _x.a, _x.b, _x.c, _x.d, _x.e, _x.f))
      length = len(self.paramValue)
      buff.write(_struct_I.pack(length))
      for val1 in self.paramValue:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 49
      (_x.mode, _x.a, _x.b, _x.c, _x.d, _x.e, _x.f,) = _get_struct_B6d().unpack(str[start:end])
      self.mode = bool(self.mode)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.paramValue = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.paramValue.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B6d = None
def _get_struct_B6d():
    global _struct_B6d
    if _struct_B6d is None:
        _struct_B6d = struct.Struct("<B6d")
    return _struct_B6d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rosdemo_v4/MovJResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MovJResponse(genpy.Message):
  _md5sum = "ca16cfbd5443ad97f6cc7ffd6bb67292"
  _type = "rosdemo_v4/MovJResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 res
"""
  __slots__ = ['res']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       res

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MovJResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.res is None:
        self.res = 0
    else:
      self.res = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.res
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.res,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.res
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.res,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
class MovJ(object):
  _type          = 'rosdemo_v4/MovJ'
  _md5sum = 'dd1dc62fc49b20e14b128b54ed54d2c0'
  _request_class  = MovJRequest
  _response_class = MovJResponse