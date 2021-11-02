import argparse
import numpy as np
import rospkg


####################################################
#THIS CODE IS TO AUGMENT YOUR HANDCRAFTED WAYPOINTS#
####################################################

parser = argparse.ArgumentParser()
parser.add_argument(
    "--track_id",
    type=int,
    required=True,
    help="which track image to construct map",
)
parser.add_argument(
    "--divide",
    type=int,
    required=False,
    default=10,
    help="equally divide each edge to add waypoints",
) #WAYPOINTS ARE AUGMENTED AT THIS SCALE


args = parser.parse_args()
file_path = "../../worlds/waypoints/track_{}_waypoints.txt".format(args.track_id) # WHERE WE GET THE RAW WAYPOINT FILE


lines = []
with open(file_path, "r") as f:
    for line in f:
        lines.append(line.split())
N = len(lines)

save_path = "../../worlds/waypoints_augmented/track_{}_aug_waypoints.txt".format(args.track_id) # WHERE THE AUGMENTED WAYPOINT PATH GOES



waypoints = []
divide=args.divide

save_file = open(save_path, "wb")
for i in range(N):
    for j in range(divide):
        next = i+1
        if i==N-1:
          next = 0
        augmented = np.array([float(lines[i][0])*(1-j/float(divide))+float(lines[next][0])*(j/float(divide)), float(lines[i][1])*(1-j/float(divide))+float(lines[next][1])*(j/float(divide))])
        np.savetxt(save_file, augmented.reshape((1,2)), fmt = "%.3e")



save_file.close()

