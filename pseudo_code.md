# Data classes/structs

```C++

Header
	double timestamp
	string frame_id

Transform
	Eigen::MatrixXd tf

TransformStamped
	Header header
	string child_frame_id
	Transform transform

Position
	double x
	double y
	double z

Rotation
	double w
	double x
	double y
	double z

Pose
	Position position
	Rotation rotation

RgbImage
	cv::Mat image		WxHx3 uint8 array of pixels

DepthImage
	cv::Mat image		WxHx1 float array of depths

CameraInfo
	int width
	int height
	cv::Mat intrinsics
	cv::Mat distortion

CameraData
	TransformStamped tf_visodom_from_camera
	CameraInfo camera_info
	RgbImage rgb
	DepthImage depth

Mask
	string label
	cv::Mat mask

FieldMaskStamped
	Header header
Mask mask

Size
	double x
	double y
	double z

SizeStamped
	Header header
	Size size

FieldDescription
	Header header
	Transform tf_fieldcenter_from_camera
	SizeStamped size

Keypoint
	Label label
	KeypointLabel keypoint_label
	float x
	float y

KeypointsStamped
	Header header
	Keypoint[] keypoints
	CameraInfo camera_info

RobotDescription
	Label label
	Group group
	Pose pose
	Size size

RobotDescriptionsStamped
	Header header
	RobotDescription[] descriptions

RobotConfig
	Label label
	Group group

VelocityCommand
	double linear_x
double linear_y
double angular_z
```

# Enums

```C++
Label
MR_STABS_MK1
	MR_STABS_MK2
	MRS_BUFF_MK1
	MRS_BUFF_MK2
	HOUSE_BOT
	OPPONENT

KeypointLabel
	MR_STABS_MK1_FRONT
	MR_STABS_MK1_BACK
	MR_STABS_MK2_FRONT
	MR_STABS_MK2_BACK
	MRS_BUFF_MK1_FRONT
	MRS_BUFF_MK1_BACK
	MRS_BUFF_MK2_FRONT
	MRS_BUFF_MK2_BACK
	HOUSE_BOT_FRONT
HOUSE_BOT_BACK

Group
	OURS
    THEIRS
```

# Interfaces

```C++
RgbdCameraInterface
	bool initialize()
	bool update()
	bool get(CameraData& data)

FieldModelInterface
	bool initialize()
	FieldMaskStamped update(RgbImage image)

FieldFilterInterface
	void reset(TransformStamped tf_visodom_from_camera)
	FieldDescription compute_field(CameraData camera_data, FieldMaskStamped field_mask)
	FieldDescription track_field(TransformStamped tf_visodom_from_camera, FieldDescription initial_description)

KeypointModelInterface
	bool initialize()
	KeypointsStamped update(RgbImage image)

RobotFilterInterface
	bool initialize(RobotConfig[] robots)
	RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field)

NavigationInterface
	bool initialize()
	VelocityCommand update(RobotDescriptionsStamped robots)

TransmitterInterface
	bool initialize()
	void update()
	void send(VelocityCommand command)
	bool did_init_button_press()
```

# Pseudo code

```C++
class Runner
    Runner(
		RobotConfig[] robot_configs,
		RgbdCameraInterface camera,
		FieldModelInterface field_model,
		FieldFilterInterface field_filter,
		KeypointModelInterface keypoint_model,
		RobotFilterInterface robot_filter,
		NavigationInterface navigation,
		TransmitterInterface transmitter,
	)
		// assign properties
        initialized = false
        initial_field_description = FieldDescription()
        
    void initialize()
        // initialize all interfaces

    void initialize_field(CameraData camera_data)
        field_filter.reset(camera_data.tf_visodom_from_camera)
        field_mask = field_model.update(camera_data.rgb)
        initial_field_description = field_filter.compute_field(camera_data, field_mask)

        robot_filter.initialize(robot_configs)
        navigation.initialize()
        initialized = true

    int run()
        while true
            if not tick()
                return 0
    
    bool tick()
        transmitter.update()
        CameraData camera_data
        if not camera.update() or not camera.get(camera_data)
            // push error to diagnostics and log
            continue
        
        if transmitter.did_init_button_press()
            initialize_field(camera_data)

        if not initialized:
            return true
        
        field_description = field_filter.track_field(camera_data.tf_visodom_from_camera, initial_field_description)
        keypoints = keypoint_model.update(camera_data.rgb)
        robots = robot_filter.update(keypoints, field_description)
        command = navigation.update(robots)
        transmitter.send(command)

int main()
    ClassConfiguration class_config = load_classes_from_config()
	RobotConfig[] robot_configs = load_robots_from_config()
	RgbdCameraInterface camera = make_rgbd_camera(class_config.camera)
	FieldModelInterface field_model = make_(class_config.field_model)
	FieldFilterInterface field_filter = make_field_filter(class_config.field_filter)
	KeypointModelInterface keypoint_model = make_keypoint_model(class_config.keypoint_model)
	RobotFilterInterface robot_filter = make_robot_filter(class_config.robot_filter)
	NavigationInterface navigation = make_navigation(class_config.navigation)
	TransmitterInterface transmitter = make_transmitter(class_config.transmitter)
    runner = Runner(
		robot_configs,
		camera,
		field_model,
		field_filter,
		keypoint_model,
		robot_filter,
		navigation,
		transmitter,
	)
    runner.initialize()
    return runner.run()
```
