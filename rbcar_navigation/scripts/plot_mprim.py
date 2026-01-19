#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther

import matplotlib.pyplot as plt
import numpy as np
import sys


def get_value(strline, name):
    if strline.find(name) < 0:
        raise Exception("File format not matching the parser expectation", name)

    return strline.replace(name, "", 1)


def get_pose(line):
    ss = line.split()
    return np.array([float(ss[0]), float(ss[1]), float(ss[2])])


class MPrim:
    def __init__(self, f):
        self.primID = int(get_value(f.readline(), "primID:"))
        self.startAngle = int(get_value(f.readline(), "startangle_c:"))
        self.endPose = get_pose(get_value(f.readline(), "endpose_c:"))
        self.cost = float(get_value(f.readline(), "additionalactioncostmult:"))
        self.nrPoses = int(get_value(f.readline(), "intermediateposes:"))
        poses = []
        for _ in range(self.nrPoses):
            poses.append(f.readline())
        self.poses = np.loadtxt(poses, delimiter=" ")
        self.cmap = plt.get_cmap("nipy_spectral")

    def plot_zero_angle(self):
        print("Plotting primID:", self.primID)
        plt.plot(self.poses[:, 0], self.poses[:, 1], label=f"primID {self.primID}", zorder=2)
        plt.scatter(self.poses[:, 0], self.poses[:, 1], s=20, zorder=3)

    def plot(self, nr_angles):
        print("Plotting primID:", self.primID)
        plt.plot(self.poses[:, 0], self.poses[:, 1], c=self.cmap(float(self.startAngle) / nr_angles))
        plt.scatter(self.poses[:, 0], self.poses[:, 1], c=[self.cmap(float(self.startAngle) / nr_angles)], s=20, zorder=3)


class MPrims:
    def __init__(self, filename):
        f = open(filename, "r")

        self.resolution = float(get_value(f.readline(), "resolution_m:"))
        self.nrAngles = int(get_value(f.readline(), "numberofangles:"))
        self.nrPrims = int(get_value(f.readline(), "totalnumberofprimitives:"))

        self.prims = []
        for _ in range(self.nrPrims):
            self.prims.append(MPrim(f))

        f.close()

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xticks(np.arange(-10, 10, step=self.resolution))
        ax.set_yticks(np.arange(-10, 10, step=self.resolution))
        for prim in self.prims[:]:
            prim.plot(self.nrAngles)
        plt.title("All Angles")
        plt.grid(True)
        plt.show()

    def plot_zero_angle(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xticks(np.arange(-10, 10, step=self.resolution))
        ax.set_yticks(np.arange(-10, 10, step=self.resolution))
        for prim in self.prims[:32]:
            prim.plot_zero_angle()
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid(True)
        plt.title("Start Angle = 0")
        plt.show()


prims = MPrims(sys.argv[1])
prims.plot_zero_angle()
