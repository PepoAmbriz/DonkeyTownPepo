import mmap
import os
import struct
import ctypes

module_path = os.path.dirname(os.path.abspath(__file__))


class MapedVar(object):
	def __init__(self,path,size):
		path = module_path+'/tmp/'+path+".mv"
		if not os.path.exists(path):
			dirname = os.path.dirname(path)
			if not os.path.exists(dirname):
				os.makedirs(dirname)
			with open(path, "w") as f:
				f.write(" "*size)
		with open(path, "r+b") as f:
			self.mm = mmap.mmap(f.fileno(),0)
		self.mm.resize(size)
		self.path = path
		self.size = size
	def read(self):
		self.mm.seek(0)
		text = self.mm.readline()
		return(text)
	def read_bytes(self,n):
		self.mm.seek(0)
		return self.mm.read(n)
	def write(self,text):
		text += " "*(self.size-len(text))
		self.mm[:len(text)] = text
	def close(self):
		self.mm.close()
	def delete(self):
		self.close()
		os.remove(self.path)
	def resize(self,new_size):
		self.mm.resize(new_size)
		self.size = size


class MapedInt(MapedVar):
	def __init__(self, path):
		super(MapedInt,self).__init__(path,4)
	def set(self, value):
		strdata = struct.pack('i',value)
		self.write(strdata)
	def get(self):
		strdata = self.read_bytes(4)
		idata = struct.unpack('i',strdata)
		return idata[0]

class MapedFloat(MapedVar):
	def __init__(self, path):
		super(MapedFloat,self).__init__(path,4)
	def set(self, value):
		strdata = struct.pack('f',value)
		self.write(strdata)
	def get(self):
		strdata = self.read_bytes(4)
		idata = struct.unpack('f',strdata)
		return idata[0]

class MapedDouble(MapedVar):
	def __init__(self, path):
		super(MapedFloat,self).__init__(path,8)
	def set(self, value):
		strdata = struct.pack('d',value)
		self.write(strdata)
	def get(self):
		strdata = self.read_bytes(8)
		idata = struct.unpack('d',strdata)
		return idata[0]

class MapedBool(MapedVar):
	def __init__(self, path):
		super(MapedBool,self).__init__(path,1)
	def set(self, value):
		strdata = struct.pack('?',value)
		self.write(strdata)
	def get(self):
		strdata = self.read_bytes(1)
		idata = struct.unpack('?',strdata)
		return idata[0]

class MapedShortInt(MapedVar):
	def __init__(self, path):
		super(MapedShortInt,self).__init__(path,2)
	def set(self, value):
		strdata = struct.pack('h',value)
		self.write(strdata)
	def get(self):
		strdata = self.read_bytes(2)
		idata = struct.unpack('h',strdata)
		return idata[0]