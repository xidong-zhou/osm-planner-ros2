# osm-planner
## First
```
cd /opt/ros/foxy/lib/python3.8/site-packages/geodesy
sudo gedit wu_point.py
```
Delete UTM coordinate conversion section
```
class WuPoint:
    """
    :class:`WuPoint` represents a map way point with associated UTM_
    information.

    :param waypt: `geographic_msgs/WayPoint`_ message.
    :param utm: Corresponding :class:`geodesy.utm.UTMPoint` object. If None
                provided, the *utm* object will be created.
 
    .. describe:: str(wu_point)
 
       :returns: String representation of :class:`WuPoint` object.
    """

    def __init__(self, waypt, utm=None):
        """Constructor.

        Collects relevant information from the way point message, and
        creates the corresponding  :class:`geodesy.utm.UTMPoint`.
        """
        self.way_pt = waypt
        self.utm = utm
```
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
###  Change frame_id
{map_frame = "base"}(plan in this frame) in osm_parser.cpp.

{self.map.header.frame_id = '/world'} in viz_osm.py.

{map = "map"}(map_frame_id) in utm_convert.cpp.

## Test
First,run planner_node and rviz.
```
ros2 launch osm_plan planner_node_launch.py 
ros2 launch osm_cartography viz_osm.launch.py
``` 
