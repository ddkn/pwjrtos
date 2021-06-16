#/usr/bin/env python3
# Copyright (c) 2021 David Kalliecharan <dave@dal.ca>

from argparse import ArgumentParser
import numpy as np
import struct
import os.path

CHAR = {
    'B' : 1,
    'H' : 2,
    'I' : 4,
    'Q' : 8,
}

def bin2data(bdata, fmt="H"):
    """Converts binary data to an unsigned type integer"""
    char_len = CHAR[fmt]
    length = len(bdata) // char_len
    data = np.array(struct.unpack(fmt * length, bdata))
    return data


def tofile(data, fname):
    if type(data) != np.ndarray:
        raise TypeError("Not a numpy.ndarray!")
    data.tofile(fname, ",", "%i")
    return None


if __name__ == "__main__":
    parser = ArgumentParser(description="Convert binary data to CSV")
    parser.add_argument('file', type=str, help="Binary data file")
                            
    args = parser.parse_args()
    
    with open(args.file, 'rb') as f:
        data = bin2data(f.read())

        fname, _ = os.path.splitext(args.file)
        tofile(data, fname + ".csv")
