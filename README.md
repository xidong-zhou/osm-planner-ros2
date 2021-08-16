# osm-planner
## init
```
pip install pyproj
sudo apt install ros-foxy-geodesy
```
## Parameter initialization
### Select Map
Change map parameter and 'lon', 'lat' in lanuch file.
```
map_file_name = 'map.osm'
origin_latitude = 28.185424
origin_longitude = 112.945468
``` 
### Determine the origin of map coordinates
View latitude and longitude data in "map.osm" file.
```
<bounds minlat="28.1825200" minlon="112.9419600" maxlat="28.1878000" maxlon="112.9489900"/>
```
Then Convert latitude and longitude data to UTM coordinates in "https://www.earthpoint.us/Convert.aspx".
And set TF conversion in launch.
```
 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['690975', '3119274', '0', '0', '0', '0', 'map', 'local_map']
        ),
```
## Test
First,run planner_node and rviz.
```
ros2 launch osm_plan planner_node_launch.py 
ros2 launch osm_cartography viz_osm.launch.py
``` 
