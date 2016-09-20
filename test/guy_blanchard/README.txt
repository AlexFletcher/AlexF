/*

Copyright (c) 2005-2016, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

VERTEX MODEL SIMULATION CODE FOLDER

This README is divided into three sections:

1. File structure
2. Compiling the code
3. Running the code

#####################################################################

1. FILE STRUCTURE 

This folder contains a collection of files which use the Chaste code 
from to recreate the results presented in our manuscript 
"Unipolar distributions of junctional Myosin II identify cell stripe 
boundaries that drive cell intercalation throughout Drosophila axis extension"

There are three folders - build, src, and apps. 

The <build> folder will contain the libraries that you compile and can be ignored.

The <src> folder contains a number of .hpp and .cpp files defining 
classes that are required for this project.
The <test> folder contains a number of .hpp files defining 'test suites', 
each corresponding to a model scenario as described in the main text of the manuscript.

######################################################################

2. COMPILING THE CODE

In order to reproduce the complete paper results we recommend performing the following steps:

a) Install version 3.4 of Chaste. 
You find instructions for this here:
https://chaste.cs.ox.ac.uk/trac/wiki/InstallGuides/InstallGuide

b) Add the following line to your Chaste hostconfig file, or edit it accordingly if it already exists:

boost_libs = ['boost_serialization', 'boost_filesystem', 'boost_program_options']

The hostconfig file is located in <YourChasteDirectory>/python/hostconfig and 
details on the chaste hostconfig system can be found here:

https://chaste.cs.ox.ac.uk/trac/wiki/ChasteGuides/HostconfigSystem

c) Copy this folder into <YourChasteDirectory>/projects/

d) Define your environment variable CHASTE_TEST_OUTPUT to be the directory
where you would like the simulation results to be saved. You can for example 
do that by adding the following line to your .bashrc file in your home directory.

export CHASTE_TEST_OUTPUT=<ADD_YOUR_FOLDER_HERE>
######################################################################

3. RUNNING THE CODE 

a) Compile and run optimized code from the project. Depending on the system you are using, 
you can do this with one of the following two commands from your Chaste main directory.

Using GNU compiler with optimized compiler flags:
scons cl=1 build=GccOptNative ./projects/<YourProjectName>/test/

Using the Intel compiler:
scons cl=1 build=Intel ./projects/<YourProjectName>/test/

It is also possible to run individual scenario simulations, e.g. as follows:
scons cl=1 build=GccOptNative ./projects/<YourProjectName>/test/TestScenario1.hpp

b) To visualize the results of each simulation, we recommend you use Paraview, which allows 
visualization of VTK files. Full details on how to do this can be found in a Chaste 'tutorial' 
wiki page here:
https://chaste.cs.ox.ac.uk/trac/wiki/UserTutorials/VisualizingWithParaview
