#!/usr/bin/env python

"""gpsemulator.py: Pretends to be a GPS receiver, NEMA protocol. Provide port name as first argument or hardcode below."""

#    gpsemulator.py
#    Copyright (C) 2022 grob6000 (https://github.com/grob6000)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

from cProfile import run
from operator import xor
from re import ASCII
import serial
import sys
import datetime

# globals
portname = "COM99"

if __name__ == "__main__":

  print ("gpsemulator.py\nCopyright (C) 2022 grob6000 (https://github.com/grob6000)\nThis program comes with ABSOLUTELY NO WARRANTY.\nThis is free software, and you are welcome to redistribute it under certain conditions.\nFor details see LICENSE at https://github.com/grob6000/dekacontroller-firmware.")

  if len(sys.argv) > 1:
    portname = sys.argv[1]
  ser = None
  try:
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port=portname
    ser.rts=0 # don't reset
    ser.dtr=0 # don't reset
    ser.timeout = 0.5
    print(ser)
    ser.open()
  except:
    print("Serial port problem, aborting. If you want to specify port in commandline, pass as first argument (e.g. python gpsemulator.py COM3)")
    quit()

  print("Running GPS Emulator on port " + portname)

  validity = "A"
  dotime = True
  locstring = "3351.908,S,15112.594,E,0.00,000.00"
  offset = 0 #seconds
  if len(sys.argv)>2:
    for v in sys.argv[2:]:
      if v.lower() == "notime":
        dotime = False
        print("Run without fix")
      elif v.lower() == "nofix":
        #dotime = False
        validity = "V"
        locstring = ",,,,,"
        print("Run without time")
      elif v.lower().startswith("offset="):
        offset=int(v.split("=",2)[1])
        print("Run with offset = {0} seconds".format(offset))
  try:
    lastsecond = -1
    inbuffer = ""
    while True:
      t = datetime.datetime.utcnow() + datetime.timedelta(seconds=offset)
      if (t.second != lastsecond):
        #print("Sending time: " + str(datetime.datetime.now()))
        lastsecond = t.second
        csum=10
        if dotime:
          body = "GPRMC,{0:02d}{1:02d}{2:02d}.000,{3},{4},{5:02d}{6:02d}{7:02d},,".format(t.hour,t.minute,t.second,validity,locstring,t.day,t.month,t.year%100)
        else:
          body = "GPRMC,,{0},{1},,,".format(validity,locstring)
        csum = 0
        for c in bytearray(body, "ASCII"):
          csum = xor(csum, c)
        msg = "${0}*{1:02X}\r\n".format(body,csum).encode("ASCII")
        print(msg)
        ser.write(msg)
      s = ""
      s = ser.read_until('\n')
      if len(s) > 0:
        print("received: ", s)
  except KeyboardInterrupt:
      ser.close()
      print("Quitting")

  
