# Message Store Map Server

This package provides tools for serving OccupancyGrid maps from the ros_datacentre. 

## Running the datacentre

All of the below assumes you have the `ros_datacentre` nodes running. Do this with:

```bash
roslaunch ros_datacentre datacentre.launch 
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