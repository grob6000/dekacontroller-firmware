from PIL import Image
import sys
import os.path
import glob

if __name__ == "__main__":

  print("Icon Generator")
  filelist = []
  imgfilename = ""
  if len(sys.argv)>1:
    print(sys.argv[1])
    imgfilename = sys.argv[1]
    filelist.append(imgfilename)
  else:
    filelist = glob.glob("*.png")
    print("Found {0} png files.".format(len(filelist)))
    #print("File '{0}' does not exist. Aborting.".format(imgfilename))

  outfile = open("icons.h", "w")
  outfile.write("// icons.h - auto generated\n")
  for f in filelist:
    if os.path.isfile(f):
      print("Converting " + os.path.abspath(f))
      image = Image.open(f)
      image = image.resize((16,16))
      image = image.convert("1")
      imdata = image.getdata()
      print(imdata)
      codestring = "static const unsigned char PROGMEM icon_{0}[] = {{\n".format(os.path.basename(f).split(".")[0])
      for r in range(0,16):
        codestring = codestring + "0b"
        for c in range(0,8):
          if imdata[r*16+c] == 0:
            codestring = codestring + "1"
          else:
            codestring = codestring + "0"
        codestring = codestring + ", 0b"
        for c in range(8,16):
          if imdata[r*16+c] == 0:
            codestring = codestring + "1"
          else:
            codestring = codestring + "0"
        codestring = codestring + ",\n"
      codestring = codestring + "};\n"
      outfile.write(codestring)
    else:
      print("Skipping {0} - not a file".format(os.path.abspath(f)))

outfile.close()