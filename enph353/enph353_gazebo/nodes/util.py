#!/usr/bin/env python2

"""
Various utilities for license plates
"""

import cv2


def imread(filename, flags=cv2.IMREAD_COLOR):
    """Wrapper around cv2.imread that throws IOError if the read fails."""
    image = cv2.imread(filename, flags)
    if image is None:
        raise IOError("Couldn't read {}".format(filename))

    return image


def imwrite(filename, img, *args, **kwargs):
    """Wrapper around cv2.imwrite that throws IOError if the write fails."""
    success = cv2.imwrite(filename, img, *args, **kwargs)
    if not success:
        raise IOError("Couldn't write {}".format(filename))

    return success
