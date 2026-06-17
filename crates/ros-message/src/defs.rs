//! Bundled standard ROS2 interface definitions.
//!
//! These let the bridge resolve the most common message/service/action types
//! out of the box without a ROS2 installation. Additional packages can be
//! loaded at runtime via [`crate::Registry::load_ament_prefix`].

use crate::registry::Registry;

/// `(fully_qualified_name, body)` pairs.
const MESSAGES: &[(&str, &str)] = &[
    // ---- builtin_interfaces ----
    ("builtin_interfaces/msg/Time", "int32 sec\nuint32 nanosec\n"),
    ("builtin_interfaces/msg/Duration", "int32 sec\nuint32 nanosec\n"),
    // ---- std_msgs ----
    ("std_msgs/msg/Header", "builtin_interfaces/Time stamp\nstring frame_id\n"),
    ("std_msgs/msg/String", "string data\n"),
    ("std_msgs/msg/Bool", "bool data\n"),
    ("std_msgs/msg/Char", "char data\n"),
    ("std_msgs/msg/Byte", "byte data\n"),
    ("std_msgs/msg/Int8", "int8 data\n"),
    ("std_msgs/msg/Int16", "int16 data\n"),
    ("std_msgs/msg/Int32", "int32 data\n"),
    ("std_msgs/msg/Int64", "int64 data\n"),
    ("std_msgs/msg/UInt8", "uint8 data\n"),
    ("std_msgs/msg/UInt16", "uint16 data\n"),
    ("std_msgs/msg/UInt32", "uint32 data\n"),
    ("std_msgs/msg/UInt64", "uint64 data\n"),
    ("std_msgs/msg/Float32", "float32 data\n"),
    ("std_msgs/msg/Float64", "float64 data\n"),
    ("std_msgs/msg/Empty", "\n"),
    ("std_msgs/msg/ColorRGBA", "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n"),
    (
        "std_msgs/msg/MultiArrayDimension",
        "string label\nuint32 size\nuint32 stride\n",
    ),
    (
        "std_msgs/msg/MultiArrayLayout",
        "MultiArrayDimension[] dim\nuint32 data_offset\n",
    ),
    (
        "std_msgs/msg/Float64MultiArray",
        "MultiArrayLayout layout\nfloat64[] data\n",
    ),
    (
        "std_msgs/msg/Float32MultiArray",
        "MultiArrayLayout layout\nfloat32[] data\n",
    ),
    (
        "std_msgs/msg/Int32MultiArray",
        "MultiArrayLayout layout\nint32[] data\n",
    ),
    (
        "std_msgs/msg/ByteMultiArray",
        "MultiArrayLayout layout\nbyte[] data\n",
    ),
    // ---- geometry_msgs ----
    ("geometry_msgs/msg/Vector3", "float64 x\nfloat64 y\nfloat64 z\n"),
    ("geometry_msgs/msg/Point", "float64 x\nfloat64 y\nfloat64 z\n"),
    ("geometry_msgs/msg/Point32", "float32 x\nfloat32 y\nfloat32 z\n"),
    (
        "geometry_msgs/msg/Quaternion",
        "float64 x\nfloat64 y\nfloat64 z\nfloat64 w 1\n",
    ),
    ("geometry_msgs/msg/Pose", "Point position\nQuaternion orientation\n"),
    ("geometry_msgs/msg/PoseStamped", "std_msgs/Header header\nPose pose\n"),
    (
        "geometry_msgs/msg/PoseWithCovariance",
        "Pose pose\nfloat64[36] covariance\n",
    ),
    (
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        "std_msgs/Header header\nPoseWithCovariance pose\n",
    ),
    ("geometry_msgs/msg/Twist", "Vector3 linear\nVector3 angular\n"),
    ("geometry_msgs/msg/TwistStamped", "std_msgs/Header header\nTwist twist\n"),
    (
        "geometry_msgs/msg/TwistWithCovariance",
        "Twist twist\nfloat64[36] covariance\n",
    ),
    ("geometry_msgs/msg/Accel", "Vector3 linear\nVector3 angular\n"),
    ("geometry_msgs/msg/Wrench", "Vector3 force\nVector3 torque\n"),
    (
        "geometry_msgs/msg/WrenchStamped",
        "std_msgs/Header header\nWrench wrench\n",
    ),
    ("geometry_msgs/msg/Transform", "Vector3 translation\nQuaternion rotation\n"),
    (
        "geometry_msgs/msg/TransformStamped",
        "std_msgs/Header header\nstring child_frame_id\nTransform transform\n",
    ),
    ("geometry_msgs/msg/Polygon", "Point32[] points\n"),
    (
        "geometry_msgs/msg/PolygonStamped",
        "std_msgs/Header header\nPolygon polygon\n",
    ),
    (
        "geometry_msgs/msg/PointStamped",
        "std_msgs/Header header\nPoint point\n",
    ),
    // ---- sensor_msgs ----
    (
        "sensor_msgs/msg/CompressedImage",
        "std_msgs/Header header\nstring format\nuint8[] data\n",
    ),
    (
        "sensor_msgs/msg/Image",
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\n",
    ),
    (
        "sensor_msgs/msg/Imu",
        "std_msgs/Header header\nQuaternion orientation\nfloat64[9] orientation_covariance\nVector3 angular_velocity\nfloat64[9] angular_velocity_covariance\nVector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n",
    ),
    (
        "sensor_msgs/msg/LaserScan",
        "std_msgs/Header header\nfloat32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\nfloat32 time_increment\nfloat32 scan_time\nfloat32 range_min\nfloat32 range_max\nfloat32[] ranges\nfloat32[] intensities\n",
    ),
    (
        "sensor_msgs/msg/JointState",
        "std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n",
    ),
    (
        "sensor_msgs/msg/NavSatStatus",
        "int8 STATUS_NO_FIX=-1\nint8 STATUS_FIX=0\nint8 STATUS_SBAS_FIX=1\nint8 STATUS_GBAS_FIX=2\nint8 status\nuint16 SERVICE_GPS=1\nuint16 SERVICE_GLONASS=2\nuint16 SERVICE_COMPASS=4\nuint16 SERVICE_GALILEO=8\nuint16 service\n",
    ),
    (
        "sensor_msgs/msg/NavSatFix",
        "std_msgs/Header header\nNavSatStatus status\nfloat64 latitude\nfloat64 longitude\nfloat64 altitude\nfloat64[9] position_covariance\nuint8 position_covariance_type\n",
    ),
    (
        "sensor_msgs/msg/PointField",
        "uint8 INT8=1\nuint8 UINT8=2\nuint8 INT16=3\nuint8 UINT16=4\nuint8 INT32=5\nuint8 UINT32=6\nuint8 FLOAT32=7\nuint8 FLOAT64=8\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n",
    ),
    (
        "sensor_msgs/msg/PointCloud2",
        "std_msgs/Header header\nuint32 height\nuint32 width\nPointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense\n",
    ),
    (
        "sensor_msgs/msg/Range",
        "std_msgs/Header header\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\nuint8 radiation_type\nfloat32 field_of_view\nfloat32 min_range\nfloat32 max_range\nfloat32 range\n",
    ),
    (
        "sensor_msgs/msg/Temperature",
        "std_msgs/Header header\nfloat64 temperature\nfloat64 variance\n",
    ),
    (
        "sensor_msgs/msg/BatteryState",
        "std_msgs/Header header\nfloat32 voltage\nfloat32 temperature\nfloat32 current\nfloat32 charge\nfloat32 capacity\nfloat32 design_capacity\nfloat32 percentage\nuint8 power_supply_status\nuint8 power_supply_health\nuint8 power_supply_technology\nbool present\nfloat32[] cell_voltage\nfloat32[] cell_temperature\nstring location\nstring serial_number\n",
    ),
    (
        "sensor_msgs/msg/RegionOfInterest",
        "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify\n",
    ),
    // ---- nav_msgs ----
    (
        "nav_msgs/msg/Odometry",
        "std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n",
    ),
    (
        "nav_msgs/msg/Path",
        "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses\n",
    ),
    // ---- unique_identifier_msgs ----
    ("unique_identifier_msgs/msg/UUID", "uint8[16] uuid\n"),
    // ---- action_msgs ----
    (
        "action_msgs/msg/GoalInfo",
        "unique_identifier_msgs/UUID goal_id\nbuiltin_interfaces/Time stamp\n",
    ),
    (
        "action_msgs/msg/GoalStatus",
        "int8 STATUS_UNKNOWN=0\nint8 STATUS_ACCEPTED=1\nint8 STATUS_EXECUTING=2\nint8 STATUS_CANCELING=3\nint8 STATUS_SUCCEEDED=4\nint8 STATUS_CANCELED=5\nint8 STATUS_ABORTED=6\nGoalInfo goal_info\nint8 status\n",
    ),
    ("action_msgs/msg/GoalStatusArray", "GoalStatus[] status_list\n"),
    // ---- diagnostic_msgs ----
    (
        "diagnostic_msgs/msg/KeyValue",
        "string key\nstring value\n",
    ),
    (
        "diagnostic_msgs/msg/DiagnosticStatus",
        "byte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\nbyte level\nstring name\nstring message\nstring hardware_id\nKeyValue[] values\n",
    ),
    (
        "diagnostic_msgs/msg/DiagnosticArray",
        "std_msgs/Header header\nDiagnosticStatus[] status\n",
    ),
    // ---- rosgraph_msgs ----
    ("rosgraph_msgs/msg/Clock", "builtin_interfaces/Time clock\n"),
];

/// `(fully_qualified_name, body)` service pairs.
const SERVICES: &[(&str, &str)] = &[
    ("std_srvs/srv/Empty", "---\n"),
    ("std_srvs/srv/SetBool", "bool data\n---\nbool success\nstring message\n"),
    ("std_srvs/srv/Trigger", "---\nbool success\nstring message\n"),
    (
        "example_interfaces/srv/AddTwoInts",
        "int64 a\nint64 b\n---\nint64 sum\n",
    ),
];

/// `(fully_qualified_name, body)` action pairs.
const ACTIONS: &[(&str, &str)] = &[
    (
        "example_interfaces/action/Fibonacci",
        "int32 order\n---\nint32[] sequence\n---\nint32[] sequence\n",
    ),
];

/// Load all bundled definitions into the registry. Definitions are static and
/// valid, so any parse failure is a programmer error and is surfaced via debug
/// assertion while being skipped in release.
pub fn load_into(reg: &mut Registry) {
    for (name, body) in MESSAGES {
        let r = reg.add_message(name, body);
        debug_assert!(r.is_ok(), "bundled message {name} failed: {r:?}");
    }
    for (name, body) in SERVICES {
        let r = reg.add_service(name, body);
        debug_assert!(r.is_ok(), "bundled service {name} failed: {r:?}");
    }
    for (name, body) in ACTIONS {
        let r = reg.add_action(name, body);
        debug_assert!(r.is_ok(), "bundled action {name} failed: {r:?}");
    }
}
