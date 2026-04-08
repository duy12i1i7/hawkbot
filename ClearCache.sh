#!/bin/bash

if /usr/bin/pgrep -f bringup_launch > /dev/null
then
    /usr/bin/pkill -f bringup_launch.py
    pgrep -f bringup_launch.py | xargs kill -9
fi


if /usr/bin/pgrep -f bringup03_launch > /dev/null
then

    /usr/bin/pkill -f bringup03_launch.py
    pgrep -f bringup03_launch.py | xargs kill -9

fi

if /usr/bin/pgrep -f bringup05_launch > /dev/null
then

    /usr/bin/pkill -f bringup05_launch.py
    pgrep -f bringup05_launch.py | xargs kill -9

fi

if /usr/bin/pgrep -f hawkbot_node > /dev/null
then

    /usr/bin/pkill -f hawkbot_node
    pgrep -f hawkbot_node | xargs kill -9

fi





