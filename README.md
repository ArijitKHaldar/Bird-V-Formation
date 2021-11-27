# Bird-V-Formation

The workable code is saved in the Swarm folder with name robotswarm.m
```
git clone https://github.com/ArijitKHaldar/Bird-V-Formation.git ~/Documents/ArijitDissertation_JU
cd ~/Documents/ArijitDissertation_JU/Swarm
```
Tested on MATLAB R2018a & R2021a on Windows 10 Pro (21H1) &
MATLAB R2018a on Arch Linux both on 64-bit architecture.

(P.S.: Higher single-core base speed facilitate faster execution than any amount of higher RAM or multi-core processors)

**Pre-requisites**
- The directory structure must be maintained for the code to work properly
    - If using `git clone`, nothing has to be done separately.
    - If manually downloading as `.zip` and extracting, make sure that you have the codes in a folder named `Swarm`.
      A folder named `Outputs` will be created in the directory where `Swarm` is placed.
      If `Outputs` folder already there, the resulting videos will be saved inside it, and nothing will be over-written.
- Install `ffmpeg` and add it to path in any OS that you use before running this code, else encoding into `.mp4` will fail.