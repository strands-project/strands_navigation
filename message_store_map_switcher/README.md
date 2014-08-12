# Message Store Map Server

This package provides tools for serving OccupancyGrid maps from the mongodb_store. 

## Running the datacentre

All of the below assumes you have the `mongodb_store` nodes running. Do this with:

```bash
roslaunch mongodb_store datacentre.launch 
```

## Adding maps to the datacentre

At the moment maps must be added using the `message_store_map_saver` executable. This loads a map described by a yaml file (using code from the main `map_server`) and then inserts the map into the message store with the name up to the last "." in the yaml file name. Usage is:

```bash
rosrun message_store_map_switcher message_store_map_saver <map.yaml>
```

E.g. 
```
rosrun message_store_map_switcher message_store_map_saver cs_lg.yaml
```

Results in a map named `cs_lg` being added to the message store. This currently uses the default database and collection (both named `message_store`).

## Running the map server

The `message_store_map_server` can be started as follows:

```bash
rosrun message_store_map_switcher message_store_map_server.py
```

This will start it without any map being published. If you want to start with a default map you can use the `-d` command line parameter:

```bash
rosrun message_store_map_switcher message_store_map_server.py -d cs_lg
```

This can be overridden with the ROS parameter "default_map". 

## Switching maps

To switch to another map from the message store, you must use the `message_store_map_switcher/SwitchMap` service at `/switch_map`. This accepts a string for the map name and returns a boolean if the map was switched. For example,

```bash
rosservice call /switch_map "cs_lg"
result: True
```

```bash
rosservice call /switch_map "not_cs_lg"
result: False
```






