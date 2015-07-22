#!/usr/bin/python
import sys;
import os;

if (len(sys.argv)!=3) :
    print "Usage ascii_to_rom.py <filename> <modulename>";
else :
    myFile = open(sys.argv[1],"r");

i = 0;
print "// auto-generated by ascii_to_rom.py from " + os.path.abspath(sys.argv[1]) + "; do not modify";
print "module " + sys.argv[2] + " #(parameter width_p=-1, addr_width_p=-1)";
print "(input  [addr_width_p-1:0] addr_i";
print ",output [width_p-1:0]      data_o";
print ");";
print "always_comb case(addr_i)"
for line in myFile.readlines() :
    line = line.strip();
    if (len(line)!=0):
        if (line[0] != "#") :
            print str(i).rjust(10)+": data_o = width_p'("+line+");";
            i = i + 1;
        else :
            print "                           // " + line;
print "default".rjust(10) + ": data_o = 'X;"
print "endcase"
print "endmodule"
