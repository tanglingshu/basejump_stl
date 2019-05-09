#!/usr/bin/python

#
# input format:
#  lines of verilog binary strings, e.g.
#    1001_10101_10011_1101
#  comments beginning with # sign
#  lines with just whitespace
#
# output format:
#  a module that implements a rom
#
# usage: bsg_ascii_to_rom.py <filename> <modulename>
#
# to set the default value of generated case statement:
#
# usage: bsg_ascii_to_rom.py <filename> <modulename> default_value  
# 
 
import sys;
import os;
import binascii;

default_value = "X"

if (len(sys.argv)==4) :
    default_value = sys.argv[3] if sys.argv[3] != "zero" else 0

if ((len(sys.argv)!=3) and (len(sys.argv)!=4)) :
    print "Usage ascii_to_rom.py <filename> <modulename>";
    exit -1

myFile = open(sys.argv[1],"r");

i = 0;
print "// auto-generated by bsg_ascii_to_rom.py from " + os.path.abspath(sys.argv[1]) + "; do not modify";
print "module " + sys.argv[2] + " #(parameter width_p=-1, addr_width_p=-1)";
print "(input  [addr_width_p-1:0] addr_i";
print ",output logic [width_p-1:0]      data_o";
print ");";
print "always_comb case(addr_i)"
all_zero = set("0_");
for line in myFile.readlines() :
    line = line.strip();
    if (len(line)!=0):
        if (line[0] != "#") :
            if (default_value == 'X' or not (set(line) <= all_zero)) :
                digits_only = filter(lambda m:m.isdigit(), str(line));

                # http://stackoverflow.com/questions/2072351/python-conversion-from-binary-string-to-hexadecimal
                hstr = '%0*X' % ((len(digits_only) + 3) // 4, int(digits_only, 2))

                print str(i).rjust(10)+": data_o = width_p ' (" + str(len(digits_only))+ "'b"+line+");"+" // 0x"+hstr;
            i = i + 1;
        else :
            print "                                 // " + line;
if (default_value != 'X') : 
    print "default".rjust(10) + ": data_o = width_p ' (%s);" % default_value
else :
    print "default".rjust(10) + ": data_o = 'X;"
print "endcase"
print "endmodule"
