import smbus 
import struct

class AsinusMotors: 
	def __init__(self,i2cbus=1,i2caddr=0x08):
		self.i2c = smbus.SMBus(i2cbus)
		self.addr = i2caddr
	@staticmethod
	def byte2float(bytes):
		fdata = []	
		for word in bytes:
			bnum = ""
			for b in word:
				bnum = bnum+struct.pack("B",b)
			fnum = struct.unpack("f",bnum) 
			fdata.append(fnum)
		return fdata
	@staticmethod
	def float2byte(fvals): 
		blist = []
		for fnum in fvals:
			bvals = list(struct.pack('f',fnum))
			for bnum in bvals: 
				blist.append(int(bnum.encode('hex'),16))
		return blist	
	def getMeasures(self): 
		bdata = self.i2c.read_i2c_block_data(self.addr,0x00,12)
		fdata = self.byte2float([bdata[0:4],bdata[4:8],bdata[8:12]]) 
		return fdata
	def setSpeeds(self,vels):
		print("Hello from setSpeeds")
		print(vels)
		blist = self.float2byte(vels)
		print(blist)
		self.i2c.write_i2c_block_data(self.addr,0x00,blist)
	def stop(self):
		self.setSpeeds([0.0,0.0])
	def close(self): 
		self.i2c.close()
