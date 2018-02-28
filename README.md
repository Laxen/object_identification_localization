## Overview
System for object identification and localization using point clouds, developed by Alexander Ganslandt and Andreas Svensson as a master's thesis at the Institute of Computer Science at the Faculty of Engineering, Lund University. For details look at our master's thesis report [NEED LINK]. 

This system identifies and localizes arbitrary objects using point clouds from a depth camera. It is intended for applications in robotics where the depth camera is mounted on a robot arm and positional data from the robot is used to merge point clouds. The system suggests where to move the camera to gain as much information as possible about the object, based on Next Best View heuristics. New objects to be identified/localized are easily added as CAD-models.

## Installation
Installation instructions for Linux can be found [here](https://github.com/Laxen/object_identification_localization/blob/master/docs/installation.md). Note that this project requires several special dependencies listed in the installation instructions.

## Usage
An overview of the different parts of the system, as well as how to compile and use them can be found [here](https://github.com/Laxen/object_identification_localization/blob/master/docs/README.md).

## License
Copyright 2018 Alexander Ganslandt and Andreas Svensson

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
