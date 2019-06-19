

# Map Server

- load map
- save map


## image format
> image用不同的像素颜色描述世界中每个cell的占用状态。白色像素表示free，黑色像素表示occupied, 其它颜色像素表示unknown。 
彩色图和灰度图都可以，但是多数是灰度图。在YAML文件中使用阈值来区分3种类别。

> 当image中像素与阈值参数比较时候，需要先按照公式occ = (255 - color_avg) / 255.0计算占用概率, 
这里color_avg是用8位数表示的来自于所有通道的平均值。例如，如果image是24-bit颜色，拥有0x0a0a0a颜色的像素，其概率是0.96，这意味着几乎完全占用。
如果像素颜色是0xeeeeee，则占用概率是0.07, 这意味着几乎没有被占用。

> 当使用ROS消息通信时候，这种占用被表示为范围[0,100]之内的一个整数, 
- 0 的意思是完全free， 
- 100 的意思是完全occupied, 
- -1 表示完全unknown。

## YAML描述文件

- image : 指定包含occupancy data的image文件路径; 可以是绝对路径，也可以是相对于YAML文件的对象路径
- resolution : 地图分辨率，单位是meters / pixel
- origin : The 2-D pose of the lower-left pixel in the map,[左上角像素值在世界坐标系的位置] 表示为 (x, y, yaw), 这里yaw是逆时针旋转角度(yaw=0意味着没有旋转)
- occupied_thresh : 像素的占用概率比该阈值大被看做完全占用
- free_thresh : 像素的占用概率比该阈值小被看做完全free
- negate： 是否反转 Whether the white/black free/occupied semantics should be reversed

- mode : trinary scale raw

/** Map mode
 *  Default: TRINARY -
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - Unknown
 *  SCALE -
 *      alpha < 1.0 - Unknown
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - f( (free_th, occ_th) ) = (0, 100)
 *          (linearly map in between values to (0,100)
 *  RAW -
 *      value = value
 */


```
image: a.pgm
resolution: 0.020000
origin: [-18.820000, -13.620000, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

```

## map-server use

```
rosrun map_server map_server mymap.yaml   ==> load map
publish :
/map_metadata
/map

static_map server  ==> 重新load map


rosrun map_server map_saver -f mymap  ==> map saver

```

## 代码解析

- 使用 YAMLCPP 解析yaml文件
- 使用 SDL_image load image map, 封装nav_msgs/OccupancyGrid

- 发布 /map_metadata  /map
- 启动服务  static_map