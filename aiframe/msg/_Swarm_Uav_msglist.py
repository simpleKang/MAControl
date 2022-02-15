# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from aiframe/Swarm_Uav_msglist.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import aiframe.msg

class Swarm_Uav_msglist(genpy.Message):
  _md5sum = "633cbed1422e3e850f0192d99770c602"
  _type = "aiframe/Swarm_Uav_msglist"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Swarm_Uav_msg[] uav_list
================================================================================
MSG: aiframe/Swarm_Uav_msg
int8 sys_id
float64 time_usec
float64 lat
float64 lon
float64 alt
float64 local_x
float64 local_y
float64 local_z
float64 v_x
float64 v_y
float64 v_z
float64 w
float64 x
float64 y
float64 z
float64 a_x
float64 a_y
float64 a_z
"""
  __slots__ = ['uav_list']
  _slot_types = ['aiframe/Swarm_Uav_msg[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       uav_list

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Swarm_Uav_msglist, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.uav_list is None:
        self.uav_list = []
    else:
      self.uav_list = []

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
      length = len(self.uav_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.uav_list:
        _x = val1
        buff.write(_get_struct_b17d().pack(_x.sys_id, _x.time_usec, _x.lat, _x.lon, _x.alt, _x.local_x, _x.local_y, _x.local_z, _x.v_x, _x.v_y, _x.v_z, _x.w, _x.x, _x.y, _x.z, _x.a_x, _x.a_y, _x.a_z))
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
      if self.uav_list is None:
        self.uav_list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.uav_list = []
      for i in range(0, length):
        val1 = aiframe.msg.Swarm_Uav_msg()
        _x = val1
        start = end
        end += 137
        (_x.sys_id, _x.time_usec, _x.lat, _x.lon, _x.alt, _x.local_x, _x.local_y, _x.local_z, _x.v_x, _x.v_y, _x.v_z, _x.w, _x.x, _x.y, _x.z, _x.a_x, _x.a_y, _x.a_z,) = _get_struct_b17d().unpack(str[start:end])
        self.uav_list.append(val1)
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
      length = len(self.uav_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.uav_list:
        _x = val1
        buff.write(_get_struct_b17d().pack(_x.sys_id, _x.time_usec, _x.lat, _x.lon, _x.alt, _x.local_x, _x.local_y, _x.local_z, _x.v_x, _x.v_y, _x.v_z, _x.w, _x.x, _x.y, _x.z, _x.a_x, _x.a_y, _x.a_z))
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
      if self.uav_list is None:
        self.uav_list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.uav_list = []
      for i in range(0, length):
        val1 = aiframe.msg.Swarm_Uav_msg()
        _x = val1
        start = end
        end += 137
        (_x.sys_id, _x.time_usec, _x.lat, _x.lon, _x.alt, _x.local_x, _x.local_y, _x.local_z, _x.v_x, _x.v_y, _x.v_z, _x.w, _x.x, _x.y, _x.z, _x.a_x, _x.a_y, _x.a_z,) = _get_struct_b17d().unpack(str[start:end])
        self.uav_list.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_b17d = None
def _get_struct_b17d():
    global _struct_b17d
    if _struct_b17d is None:
        _struct_b17d = struct.Struct("<b17d")
    return _struct_b17d
