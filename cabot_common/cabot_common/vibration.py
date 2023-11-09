#!/usr/bin/env python

# Copyright (c) 2023  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

UNKNOWN = 0
LEFT_TURN = 1
RIGHT_TURN = 2
LEFT_DEV = 3
RIGHT_DEV = 4
FRONT = 5
LEFT_ABOUT_TURN = 6
RIGHT_ABOUT_TURN = 7
BUTTON_CLICK = 8
BUTTON_HOLDDOWN = 9
STIMULI_COUNT = 10

stimuli_names = ["unknown", "left_turn", "right_turn", "left_dev", "right_dev",
                 "front", "left_about_turn", "right_about_turn", "button_click", "button_holddown"]


def get_name(stimulus):
    return stimuli_names[stimulus]
