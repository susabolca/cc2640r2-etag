import os
import struct

def flatten_list(nested_list):
    flat_list = [] 
    for item in nested_list:
        if isinstance(item, list): 
            flat_list.extend(flatten_list(item))
        else:
            flat_list.append(item)
    return flat_list

def pkg(fmt, num):
    a = struct.pack(fmt, num)
    return [ x for x in a ]

def rgb(r, g, b):
    return [b, g, r]

def genTestBmp(fpath):
    width = 296
    height = 128

    # header
    out = [
        0x42, 0x4D,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x36, 0x00, 0x00, 0x00,     # pixel data offset at 0x36
    ]

    out += [
        0x28, 0x00, 0x00, 0x00,     # header size 40 bytes
        pkg('<I', width),
        pkg('<I', height),
        0x01, 0x00,                 # plane 1
        0x18, 0x00,                 # bpp 24 bits
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    ]

    for y in range(0, height):
        for x in range(0, width):
            bg = ((x+1)//33)*32
            bg = bg - 1 if bg else 0
            if y > height/2:
                out += rgb(255, bg, bg)
            else:
                out += rgb(bg, bg, bg)
        # TBD: pad for 4 bytes alignment.

    with open(fpath, 'wb') as f:
        f.write(bytes(flatten_list(out)))

if __name__ == "__main__":
    genTestBmp('test.bmp')
