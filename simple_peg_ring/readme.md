# SIMPLE PEG AND RING 

Inside simple_peg_ring folder there are :
- coppelia_scene: dvrk scene
- script: simple_peg_ring code

## EXAMPLES CODE 
you can find example inside dvrk-ros / dvrk_python  / scripts folders

## HOW TO RUN 

Open the coppelia scene.

Then, in terminal one :

``` bash
rosrun dvrk_robot dvrk_console_json -j ~/dvrk_latest_ws/src/sawIntuitiveResearchKit/share/console/console-full-system-simulated.json 
```
- POWER ON
- HOME


in a second terminal : 

``` bash
python3 simple_peg_ring.py 
```