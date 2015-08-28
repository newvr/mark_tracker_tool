#!/bin/bash
#
# Script to launch a pepper from a launchfile with the corresponding IP env variable
#
# Fres Scarlett
#
#
# Use: ./add_robot.sh [namespace] [IP]
#

function showHelp(){
    echo
    echo "This script change the NAO_IP of the pepper, "
    echo "Place it in the 'scripts' folder of your catkin package"
    echo "and make sure that the file is executable (chmod +x add_robot.sh)"
    echo
    echo "Run it from command line:"
    echo
    echo "Use: ./add_robot.sh [namespace] [IP] "
    echo
    echo "Or run it from another roslaunch file:"
    echo
    echo '<launch>'
    echo '  <node pkg="my_package" type="add_robot.sh"'
    echo '    args="monica 10.0.206.111"'
    echo '    name="timed_roslaunch" output="screen">'
    echo '  </node>'
    echo '</launch>'
}

if [ "$1" = "-h" ]; then
    showHelp
else
    #echo "start wait for $1 seconds"
    #sleep $1
echo "jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj"
echo "$#" $#
echo " dollar 1 "
echo $1
echo " dollar 2 "
echo $2
echo " finiini "

    if test "$#" -eq 3; then
        export NAO_IP=$1
        roslaunch pepper_bringup pepper_full.launch

    else
        echo "eeeeeeeeeeeeeelse"
        export NAO_IP=$2
        roslaunch mark_tracker_tools sca_pepper.launch namespace:=$1
    fi
fi
