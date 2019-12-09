<p>
<img src="https://img.shields.io/github/repo-size/UofA-EEE-LAUS/3D-modelling" alt="repo size">
<img src="https://img.shields.io/github/languages/top/UofA-EEE-LAUS/3D-modelling" alt="code language">
<img src="https://img.shields.io/badge/platform-Win10%201809-blue" alt="platform">
</p>

# 3D-modelling
<img src="https://upload.wikimedia.org/wikipedia/en/thumb/c/ca/University-of-Adelaide-Logo.svg/220px-University-of-Adelaide-Logo.svg.png" alt="uni logo">

***University of Adelaide Summer Research Internship 2019***


## About
This repository is the work of the 2019 3D Modelling team.\
We are developing the simulation and testing environment for the University of Adelaide Summer Research Internship.\
For more detailed information, please see each model domain in the [Wiki](https://github.com/UofA-EEE-LAUS/3D-modelling/wiki)

## Installation 
### Virtual Robot Experimentation Platform (V-REP)
This project uses V-REP release 3.6.2, an open source robotics prototyping and simulation software package.\
As of 26/11/2019, V-REP has been superseeded by CopelliaSim.\
V-REP is still downloadable at the time of writing and all .ttt files should be forwards compatible with CopelliaSim.\
Available for free at http://www.coppeliarobotics.com/

### MATLAB
For the RemoteAPI to V-REP, this project has chosen MATLAB.\
The project is currently using release 2019a, however 2019b should be used soon.\
Available at: https://mathworks.com/products/matlab.html

## Instructions
### RemoteAPI
**V-REP**\
In V-REP, ensure that the line:  
`
simRemoteApi.start(19999)
`
is inside a non-threaded child script, under `sysCall_init()`  
(this runs once and starts the internal server for MATLAB to connect to)

**MATLAB**\
Ensure the files inside of  
```
C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab
C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib\lib\Windows\64Bit
```
are copied into the current MATLAB workspace.

For more information on API functions, please see:\
http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
