# Debugging Information

### In this order
- Is the node running?
- Is it receiving camera frames?
- Is it receiving camera intrinsics?
- Is OpenCV seeing usable image data?
- Is Canny detecting edges?
- Is Hough detecting lines? 
- Are your slpe filters deleting all lines? 
- Are points being projected into camera frame? 
- Is TF working from camera -> map? 
- Is /lane_path actually publishing data? 
- Can RViz display it? 

### Run and build

Build with: 

```[python]
colcon build --packages-select transpose_lane_lines
source install/setup.bash
```

Run with:

```[python]
ros2 run transpose_lane_lines transpose_lines
```

### 1) Verify the node actually starts

When the script starts, you should see 
~~~
[INFO] [lane_detector]: LaneDetector node started
~~~
If not, either:

- Your script isn't launching
- or Your ROS2 package entry point is wrong

### 2) Verify your topics exist

Run 
```[python]
ros2 topic list
```
You should see: 

```
/routecam/image_raw
/routecam/camera_info
/tf
/tf_static
/lane_path
```

If /routecam/image_raw is missing, your camera node isn’t publishing

If /routecam/camera_info is missing, you’ll never get intrinsics

### 3) Confirm image data is actually arriving

Expected:

You should see 'Image received' spam whenever a frame arrives

If you don't:

- wrong topic name
- camera node not publishing
- QoS mismatch (possible in ROS2 camera streams)

If image callback still doesn't trigger: likely QoS issue

- Many camera publishers use sensor data QoS, not the default queue-based QoS

To Fix (if needed):

Import:
```[python]
from rclpy.qos import qos_profile_sensor_data
```
Then change camera info to:
```[python]
self.create_subscription(
    Image,
    '/routecam/image_raw',
    self.imgae_callback,
    qos_profile_sensor_data
)
```
Also change camera info to:
```[python]
self.create_subscription(
    CameraInfo,
    '/routecam/camera_info',
    self.camera_info_callback,
    qos_profile_sensor_data
)
```
This is a very common ROS2 issue.
- if your callback isn't firing, this is one of the first things you should suspect

### 4) Verify camera info arrives

Expected:

You should see one or more logs like:
```[markdown]
Camera intrinsics loaded: fx=..., fy=..., cx=..., cy=...
```
If not:

- wrong topic
- QoS mismatch
- camera driver not publishing camera_info

### 5) Confirm the image format is valid

Right now you do:

frame = self.bridge.imgmsg_to_cv2(msg)
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

That can fail if the image is not BGR.

Add this debug:
self.get_logger().info(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")
If you get errors:

Try forcing encoding:

frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

If your camera is grayscale already, then use:

frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
gray = frame
STEP 6 — Confirm Canny is actually finding edges

Right after:

edges = cv.Canny(gray, 50, 150)

Add:

edge_count = np.count_nonzero(edges)
self.get_logger().info(f"Edge pixels detected: {edge_count}")
Interpretation:
0 or very low → your thresholds are too aggressive or image is bad
thousands → good

If this is near zero, your line detection will fail downstream.

STEP 7 — Confirm Hough is finding lines

Right after cv.HoughLinesP(...), add:

if lines is None:
    self.get_logger().warn("Hough detected NO lines")
else:
    self.get_logger().info(f"Hough detected {len(lines)} raw lines")
If lines is None:

The issue is not Nav2, TF, or projection.

It means:

Canny isn’t finding useful edges
Hough parameters are too strict
image doesn’t contain detectable lane edges
STEP 8 — Check whether your slope filter is deleting everything

Inside your loop, uncomment and improve this:

self.get_logger().info(f"Detected raw slope: {slope:.2f}")

Then also add:

if abs(slope) < self.config["line_min_slope"] or \
   abs(slope) > self.config["line_max_slope"]:
    self.get_logger().info(f"Rejected line with slope {slope:.2f}")
    continue
Why this matters:

Sometimes Hough finds lines, but your filtering removes all of them.

That would make it look like “line detection is broken,” when the real problem is just filtering.

STEP 9 — Confirm pixel_to_map() is actually being called

Add this before:

pose1 = self.pixel_to_map(x1, y1)

Put:

self.get_logger().info(f"Projecting line endpoints: ({x1}, {y1}) -> ({x2}, {y2})")

If you never see this:

your slope filter is rejecting everything
or lines is None
STEP 10 — Confirm 3D projection math is running

You already have this:

self.get_logger().info(f"Pixel ({u}, {v}) projected to camera frame: X={X_cam:.2f}, Y={Y_cam:.2f}, Z={Z_cam:.2f}")

That’s good.

What to watch for:

If values are absurd like:

X = 80 meters
Y = -200 meters

then:

wrong intrinsics
bad pixel values
fixed Z projection is stretching too much
STEP 11 — TF is one of your biggest likely failure points

You currently do:

point_cam.header.frame_id = "camera_link"

But your real camera frame may not be camera_link.

This is a very common break point.

Check the actual image frame ID

Inside image_callback(), add:

self.get_logger().info(f"Image frame_id: {msg.header.frame_id}")

Inside camera_info_callback(), add:

self.get_logger().info(f"CameraInfo frame_id: {msg.header.frame_id}")
If your frame is something like:
routecam_optical_frame

but you hardcoded:

point_cam.header.frame_id = "camera_link"

then TF may fail.

Better:

Store the real frame:

In __init__:

self.camera_frame = None

In camera_info_callback():

self.camera_frame = msg.header.frame_id

Then in pixel_to_map():

point_cam.header.frame_id = self.camera_frame if self.camera_frame else "camera_link"

That is much safer.

STEP 12 — Debug TF directly from terminal

Run:

ros2 run tf2_ros tf2_echo map camera_link

Or if your frame is different:

ros2 run tf2_ros tf2_echo map routecam_optical_frame
If this fails:

Then your transform does not exist.

And if TF does not exist:

pixel_to_map() will always fail
your lane_path will stay empty

This is extremely likely if you’re using a real robot or simulator and haven’t verified frames yet.

STEP 13 — Confirm /lane_path is publishing actual content

Run:

ros2 topic echo /lane_path
Expected:

You should see poses: with entries.

If you only see:

poses: []

then one of these is happening:

no lines detected
slope filter removes all lines
projection fails
TF fails
STEP 14 — Visualize in RViz2

Open RViz2 and set:

Fixed Frame = map
Add Path
Topic = /lane_path

If nothing appears:

check /lane_path content with ros2 topic echo
if data exists, it may just be off-map or bad coordinates