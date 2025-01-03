# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot_bringup/InverseSolutionRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class InverseSolutionRequest(genpy.Message):
  _md5sum = "a56434a738fb69f7e97a6dc9a3bb7f8d"
  _type = "dobot_bringup/InverseSolutionRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 offset1
float64 offset2
float64 offset3
float64 offset4
float64 offset5
float64 offset6
int32 user
int32 tool
int32 isJointNear
string JointNear
"""
  __slots__ = ['offset1','offset2','offset3','offset4','offset5','offset6','user','tool','isJointNear','JointNear']
  _slot_types = ['float64','float64','float64','float64','float64','float64','int32','int32','int32','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       offset1,offset2,offset3,offset4,offset5,offset6,user,tool,isJointNear,JointNear

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InverseSolutionRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.offset1 is None:
        self.offset1 = 0.
      if self.offset2 is None:
        self.offset2 = 0.
      if self.offset3 is None:
        self.offset3 = 0.
      if self.offset4 is None:
        self.offset4 = 0.
      if self.offset5 is None:
        self.offset5 = 0.
      if self.offset6 is None:
        self.offset6 = 0.
      if self.user is None:
        self.user = 0
      if self.tool is None:
        self.tool = 0
      if self.isJointNear is None:
        self.isJointNear = 0
      if self.JointNear is None:
        self.JointNear = ''
    else:
      self.offset1 = 0.
      self.offset2 = 0.
      self.offset3 = 0.
      self.offset4 = 0.
      self.offset5 = 0.
      self.offset6 = 0.
      self.user = 0
      self.tool = 0
      self.isJointNear = 0
      self.JointNear = ''

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
      buff.write(_get_struct_6d3i().pack(_x.offset1, _x.offset2, _x.offset3, _x.offset4, _x.offset5, _x.offset6, _x.user, _x.tool, _x.isJointNear))
      _x = self.JointNear
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      end += 60
      (_x.offset1, _x.offset2, _x.offset3, _x.offset4, _x.offset5, _x.offset6, _x.user, _x.tool, _x.isJointNear,) = _get_struct_6d3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.JointNear = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.JointNear = str[start:end]
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
      buff.write(_get_struct_6d3i().pack(_x.offset1, _x.offset2, _x.offset3, _x.offset4, _x.offset5, _x.offset6, _x.user, _x.tool, _x.isJointNear))
      _x = self.JointNear
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      end += 60
      (_x.offset1, _x.offset2, _x.offset3, _x.offset4, _x.offset5, _x.offset6, _x.user, _x.tool, _x.isJointNear,) = _get_struct_6d3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.JointNear = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.JointNear = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d3i = None
def _get_struct_6d3i():
    global _struct_6d3i
    if _struct_6d3i is None:
        _struct_6d3i = struct.Struct("<6d3i")
    return _struct_6d3i
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot_bringup/InverseSolutionResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class InverseSolutionResponse(genpy.Message):
  _md5sum = "ca16cfbd5443ad97f6cc7ffd6bb67292"
  _type = "dobot_bringup/InverseSolutionResponse"
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
      super(InverseSolutionResponse, self).__init__(*args, **kwds)
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
class InverseSolution(object):
  _type          = 'dobot_bringup/InverseSolution'
  _md5sum = 'ca1d2fdd189adb0efab2e962b8538e82'
  _request_class  = InverseSolutionRequest
  _response_class = InverseSolutionResponse
