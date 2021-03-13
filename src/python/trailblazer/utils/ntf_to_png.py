# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import argparse
from osgeo import osr, gdal

def main(ntf_fname=None, png_fname=None):
    parser = argparse.ArgumentParser(description='Randomly drop detections given a probability of detection')
    parser.add_argument('--ntf_fname',
                        help='ntf file to convert',
                        type=str,
                        default="./net_quantico/network.ntf")
    parser.add_argument('--png_fname',
                        help='png file to make',
                        type=str,
                        default="./net_quantico/network.png")
    args = parser.parse_args()
    if ntf_fname is None:
        ntf_fname = args.ntf_fname
    if png_fname is None:
        png_fname = args.png_fname

    ds = gdal.Open(ntf_fname, gdal.GA_ReadOnly)
    if ds is None:
        raise OSError()



if __name__ == '__main__':
    main()