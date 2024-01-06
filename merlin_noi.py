#!/usr/bin/python
#
# merlin_noi.py: Parse merlin32 assembler Output.txt files
# to extract debug information for use by NoICE
#
# Written by John Hartman
#
# Version 2.0
#
import sys
import string
import argparse

# S1 bytecount address data checksum
#   bytecount includes address, data, and checksum
#   checksum is LSB of one's complement of sum of bytecount, address, data, and checksum

# The first-gen "Boot in RAM" monitor locates in RAM at 0F800 - 0FFFF
# This equate warns for .pgz segments in that range.
# if the monitor moves, this could be moved to a .ini file
monitor_base = 0x0F800
monitor_top  = 0x0FFFF

#=============================================================================
# Dump a block of data as S-records
def dump_hex_file( a_outfile, a_address, a_length, a_offset, a_data ):
    # Default address for PGX, or PGZ without a zero-size segment
    if (a_address <= monitor_top) and (monitor_base < a_address + a_length):
        print( 'FATAL: Memory segment {addr:06X} {len:04X} overlaps the monitor: not output'.format(addr=a_address, len=a_length) )
        return

    print( 'Memory segment {addr:06X} {len:05X}'.format(addr=a_address, len=a_length) )

    # TODO: NoICE ignores checksum, so for a quick hack we use 00
    if a_length == 0:
        # start address
        a_outfile.write('S804{addr:06X}00\n'.format(addr=a_address))
    else:
        while a_length > 0:
            chunklen = 32 if a_length >= 32 else a_length
            str = 'S2{len:02X}{addr:06X}'.format(len=chunklen+4, addr=a_address)

            for x in range(0, chunklen):
                str += '{val:02X}'.format(val=a_data[a_offset + x])

            a_outfile.write(str + '00\n')
            a_address += chunklen
            a_length  -= chunklen
            a_offset  += chunklen

#=============================================================================
#
def main():
    parser = argparse.ArgumentParser(description =
        'Parse merlin32 assembler output file.pgz and file.pgz_Output.txt' +
        'to extract debug information for use by NoICE.'
    )
    parser.add_argument('infile',
                         help='the output pgz or pgx file from merlin32 -V')
    ns = parser.parse_args()

    # Dump the code as S-records, and harvest the start address
    start_address = 0
    with open(ns.infile,"rb") as codefile:
        with open(ns.infile + '.s19',"w") as hexfile:
            data = codefile.read()
            if data[0] == ord('P'):
                # PGX: single segment with 32-bit addresses
                ix = 4   # skip PGX and CPU type bytes
                segaddr = data[ix+0] + (data[ix+1] << 8) + (data[ix+2] << 16) + (data[ix+3] << 24)
                seglen  = len(data) - 8
                ix += 4
                dump_hex_file( hexfile, segaddr, seglen, ix, data )
                # Dummy to generate a start-address record for the segment
                dump_hex_file( hexfile, segaddr, 0, ix, data )
                start_address = segaddr

            elif data[0] == ord('Z'):
                # Big Z: multiple segments with 24-bit addresses
                ix = 1
                while ix < len(data):
                    segaddr = data[ix+0] + (data[ix+1] << 8) + (data[ix+2] << 16)
                    seglen  = data[ix+3] + (data[ix+4] << 8) + (data[ix+5] << 16)
                    ix += 6
                    dump_hex_file( hexfile, segaddr, seglen, ix, data )
                    ix += seglen
                    if seglen == 0:
                        start_address = segaddr

            elif data[0] == ord('z'):
                # Small z: multiple segments with 32-bit addresses
                ix = 1
                while ix < len(data):
                    segaddr = data[ix+0] + (data[ix+1] << 8) + (data[ix+2] << 16) + (data[ix+3] << 24)
                    seglen  = data[ix+4] + (data[ix+5] << 8) + (data[ix+6] << 16) + (data[ix+7] << 24)
                    ix += 8
                    dump_hex_file( hexfile, segaddr, seglen, ix, data )
                    ix += seglen
                    if seglen == 0:
                        start_address = segaddr

            else:
                print( 'unknown binary file beginning with {data}'.format(data=data[0]))


    # Generate symbol and source line definitions
    with open(ns.infile + '_Output.txt',"r") as infile:
        with open(ns.infile + '.noi',"w") as outfile:
            types_with_code_labels = ['Empty', 'Code']
            types_with_data_labels = ['Dum', 'Data']
            current_file = ''

            # Delete obsolete debug info (based on file or timestamp change)
            outfile.write('LASTFILELOADED\n')

            while True:
                line = infile.readline()
                if not line:
                    break
                
                fields = line.split('|')
                if len(fields) > 7:
                    stuff = fields[1].strip().split()
                    if (len(stuff) >= 3) and (stuff[0] != '#'):
                        # File Line
                        f_file = stuff[1]
                        f_line = stuff[2]

                        # Line Type
                        f_type = fields[2].strip()

                        # Size
                        f_size = int(fields[5].strip())

                        # Address  Object Code
                        # TODO: handle mapped addressing
                        stuff = fields[6].strip().split()
                        f_address = stuff[0].replace('/', ':')

                        # Source Code
                        f_sourcetext = fields[7]

                        if f_file != current_file:
                            outfile.write('file "{fname}"\n'.format(fname=f_file))
                            current_file = f_file

                        if f_type == 'Code':
                            outfile.write('line {fline} {addr}\n'.format(fline=f_line, addr=f_address))

                        if (len(f_sourcetext) > 1) and (f_sourcetext[1] != ' ') and (f_sourcetext[1] != ';'):
                            # Line may have a label
                            stuff = f_sourcetext[1:].split()
                            if f_type in types_with_code_labels:
                                outfile.write('def {name} {addr}\n'.format(name=stuff[0], addr=f_address))

                            elif f_type in types_with_data_labels:
                                typespec = ''
                                if f_size == 2:
                                    typespec = '%X16'
                                elif f_size == 3:
                                    typespec = '%X24'
                                elif f_size == 4:
                                    typespec = '%X32'
                                elif f_size > 4:
                                    typespec = '%X08[{size}.]'.format(size=f_size)

                                outfile.write('def {name} {addr} {type}\n'.format(
                                    name=stuff[0], addr=f_address, type=typespec))

                            elif (f_type == 'Equivalence') and (stuff[2][0] == '$'):
                                # Accept =, equ, or EQU
                                try:
                                    value = int(stuff[2][1:], 16)
                                    if (value >= 0xC000) and (value <= 0xDFFF):
                                        outfile.write('def {name} 0x{val:04X}\n'.format(
                                            name=stuff[0], val=value))
                                    elif (value >= 0xE000) and (value <= 0xFFFF):
                                        print('WARNING: symbol {name} 0x{val:04X} may indicate kernel usage\n'.format(
                                            name=stuff[0], val=value))
                                    
                                except ValueError:
                                    # Ignore this value
                                    continue

            if current_file != '':
                outfile.write('endfile\n')

            outfile.write('load "{fname}.s19"\n'.format(fname=ns.infile))
            if start_address != 0:
                outfile.write('reg pc 0x{val:X}\n'.format(val=start_address))
            outfile.write('mode 2\n')
            outfile.write('s pc')

if __name__ == "__main__":
   main()
