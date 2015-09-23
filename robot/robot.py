#!/usr/bin/env python3

import wpilib
from os.path import dirname, abspath
from yeti import YetiRobot


class FlatMountainBot(YetiRobot):
    config_dir = abspath(dirname(__file__))

if __name__ == "__main__":
    wpilib.run(FlatMountainBot)

