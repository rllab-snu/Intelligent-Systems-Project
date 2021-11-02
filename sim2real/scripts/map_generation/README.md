# rllab-racing
```bash
conda activate py36 # any py3 environment
(py36) pip install -r requirements.txt
# for track_i.png, set argument as --track-id i
(py36) python get_points.py --track-id 1
# after creating text file for a track, execute map_generator code to construct a map.
# make sure to switch the virtual environment to python2.7 env.
cd ..
python map_generator.py --track-id 1
```
track images should be placed in track_img folder.  
track_\*.txt, track_\*\_waypoints.txt and track_\*.world  
all three files will be generated in sim2real/worlds/ directory.

## How to generate map and waypoints using GUI
1) Click points so they can follow the outer loop of race track. After you click the last point, press Space to end the loop.
2) Click points so they can follow the inner loop of race track. After you click the last point, press Space to end the loop.
3) Click points for the waypoints of race track. After you click the last point, press s to end.

