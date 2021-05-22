# README #

## Update ##

使用以下配置进行routerless配置下的仿真：
```
--routing-algorithm=custom
--if_routerless=1 
```

TODO: 非DNN traffic中average_packet_queueing_latency与原始不一致

## Origin ##
Last Updated: November 27, 2020

This is updated Georgia Tech internal gem5 repository used by Tushar Krishna (http://tusharkrishna.ece.gatech.edu) for his Interconnection Networks course.
This is an old version of the repo (from Jan 2018) and is primarily used to build Garnet2.0 in a standalone manner.

## How to Build ##
scons build/Garnet_standalone/gem5.opt -j X

X - num cores of your processor + 1

## Example Run Command ##
See my_scripts/run_example.sh

## NoC Labs ##
http://tusharkrishna.ece.gatech.edu/teaching/icn_s20/

## More details ##
http://tusharkrishna.ece.gatech.edu/teaching/garnet_gt/

(Ignore the my_scripts/build_Garnet_standlone.sh command as that is specific to the RHEL setup at the GT ECE servers).
 
## Useful links: ##
http://www.gem5.org/Garnet_Synthetic_Traffic

http://www.gem5.org/Garnet2.0

## Exectuing Garnet2.0 with Circulants ##
To use garnet2.0 with circulants you can see plan.py.
Before executing Garnet2.0 you have build it.

To use script you have to specify root password in passw variable.

It is circulant flits's paths for specific circulant in folder circ_routes.
In each file it is specified, path for each pair start, destination for each node.
Paths are divided by ";" in file.
Each node in path is divided by "," in file.
