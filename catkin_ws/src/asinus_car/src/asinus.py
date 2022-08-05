#import smbus 
import struct
import pylibi2c


class AsinusMotors: 
	def __init__(self,i2cbus='/dev/i2c-1',i2caddr=0x08):
		self.i2c = pylibi2c.I2CDevice(i2cbus, i2caddr)
		self.addr = i2caddr	
	def getMeasures(self): 
		bdata = self.i2c.ioctl_read(0x0,12)
		fdata = struct.unpack('3f',bdata)
		return fdata
	def setSpeeds(self,vels):
		blist = bytes(struct.pack('ff',vels[0],vels[1]))
		size = self.i2c.write(0x40044000,blist)
	def stop(self):
		self.setSpeeds([0.0,0.0])
	def close(self): 
		self.i2c.close()
