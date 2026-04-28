# Pseudo code

```cpp
class Runner
    Runner(
		RgbdCameraInterface camera,
		MaskModelInterface field_model,
		RobotBlobModelInterface robot_blob_model,
		FieldFilterInterface field_filter,
		KeypointModelInterface keypoint_model,
		RobotFilterInterface robot_filter,
		NavigationInterface navigation,
        TargetSelectorInterface target_selector,
		TransmitterInterface transmitter,
        PublisherInterface publisher,
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

        robot_filter.initialize(opponent_count)
        navigation.initialize()
        initialized = true

    int run()
        while true
            if not tick()
                return 0

    bool tick()
        command_feedback = transmitter.update()
        CameraData camera_data
        if not camera.get(camera_data)
            // push error to diagnostics and log
            continue

        if transmitter.did_init_button_press()
            initialize_field(camera_data)

        if not initialized:
            return true

        field_description = field_filter.track_field(camera_data.tf_visodom_from_camera, initial_field_description)
        keypoints = keypoint_model.update(camera_data.rgb)
        robot_blob_keypoints = robot_blob_model.update(camera_data.rgb)
        robots = robot_filter.update(
            keypoints,
            field_description,
            camera_data.camera_info,
            robot_blob_keypoints,
            command_feedback
        )
        selected_target = target_selector.get_target(robots, field_description)
        if selected_target.has_value():
            target = selected_target.value()
        // Add target overrides here
        command = navigation.update(robots, field_description, target)
        transmitter.send(command)

int main()
    ClassConfiguration class_config = load_classes_from_config()
	RgbdCameraInterface camera = make_rgbd_camera(class_config.camera)
	MaskModelInterface field_model = make_mask_model(class_config.field_model)
	RobotBlobModelInterface robot_blob_model = make_robot_blob_model(class_config.robot_mask_model)
	FieldFilterInterface field_filter = make_field_filter(class_config.field_filter)
	KeypointModelInterface keypoint_model = make_keypoint_model(class_config.keypoint_model)
	RobotFilterInterface robot_filter = make_robot_filter(class_config.robot_filter)
    TargetSelectorInterface target_selector = make_target_selector(class_config.target_selector)
	NavigationInterface navigation = make_navigation(class_config.navigation)
	TransmitterInterface transmitter = make_transmitter(class_config.transmitter)
    PublisherInterface publisher = make_publisher(class_config.publisher)
    runner = Runner(
		camera,
		field_model,
        robot_blob_model,
		field_filter,
		keypoint_model,
		robot_filter,
		navigation,
        target_selector,
		transmitter,
        publisher,
	)
    runner.initialize()
    return runner.run()
```

# Interfaces

```cpp
RgbdCameraInterface
	bool initialize()
	void cancel_initialize()
	bool get(CameraData& data, bool get_depth=false)
	bool should_close()
	bool set_svo_recording_enabled(bool enabled)
	bool is_svo_recording_enabled() const

MaskModelInterface
	bool initialize()
	MaskStamped update(RgbImage image)

FieldFilterInterface
	void reset(TransformStamped tf_visodom_from_camera)
	shared_ptr<FieldDescriptionWithInlierPoints> compute_field(const CameraData& camera_data, const MaskStamped& field_mask)
	FieldDescription track_field(TransformStamped tf_visodom_from_camera, shared_ptr<FieldDescriptionWithInlierPoints> initial_description)

KeypointModelInterface
	bool initialize()
	KeypointsStamped update(RgbImage image)

RobotBlobModelInterface
	bool initialize()
	KeypointsStamped update(RgbImage image)

RobotFilterInterface
	bool initialize(int opponent_count)
	RobotDescriptionsStamped update(
		KeypointsStamped keypoints,
		FieldDescription field,
		CameraInfo camera_info,
		KeypointsStamped robot_blob_keypoints,
		CommandFeedback command_feedback
	)

TargetSelectorInterface
    optional<TargetSelection> get_target(const RobotDescriptionsStamped& robots, const FieldDescription& field)

NavigationInterface
	bool initialize()
	VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field, const TargetSelection& target)
	optional<NavigationPathSegment> get_last_path() const

TransmitterInterface
	bool initialize()
	CommandFeedback update()
	void send(VelocityCommand command)
	bool did_init_button_press()
	bool is_connected() const
	void enable()
	void disable()

PublisherInterface
    void publish_camera_data(const CameraData& data)
    void publish_field_mask(const MaskStamped& field_mask, const RgbImage& image)
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints& field)
    void publish_field_description(
        const FieldDescription& field_description,
        const FieldDescriptionWithInlierPoints& initial_field_description
    )
    void publish_robots(const RobotDescriptionsStamped& robots)
    void publish_navigation(const NavigationVisualization& nav)

DiagnosticsBackend
    void receive(const vector<DiagnosticStatusSnapshot>& snapshots)
```

# Data classes/structs

```cpp

Header
	double stamp
	FrameId frame_id

Transform
	Eigen::MatrixXd tf

TransformStamped
	Header header
	FrameId child_frame_id
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

Pose2D
	double x
	double y
	double yaw

RgbImage
	Header header
	cv::Mat image		WxHx3 uint8 array of pixels

DepthImage
	Header header
	cv::Mat image		WxHx1 float array of depths

CameraInfo
	Header header
	int width
	int height
	cv::Mat intrinsics
	cv::Mat distortion

CameraData
	TransformStamped tf_visodom_from_camera
	bool tracking_ok
	CameraInfo camera_info
	RgbImage rgb
	DepthImage depth
	Pose2D[] ground_truth_poses

Mask
	Label label
	cv::Mat mask

MaskStamped
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
	FrameId child_frame_id
	Transform tf_camera_from_fieldcenter
	SizeStamped size

PointCloud
	Header header
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud

FieldDescriptionWithInlierPoints : FieldDescription
	PointCloud inlier_points

Keypoint
	Label label
	KeypointLabel keypoint_label
	double x
	double y
	double confidence
	int detection_index

KeypointsStamped
	Header header
	Keypoint[] keypoints

Velocity2D
	double vx
	double vy
	double omega

RobotDescription
	FrameId frame_id
	Label label
	Group group
	Pose pose
	Size size
	Position[] keypoints
	Velocity2D velocity
	bool is_stale

RobotDescriptionsStamped
	Header header
	RobotDescription[] descriptions

RobotConfig
	Label label
	Group group

TargetSelection
    Pose2D pose
    Label label

VelocityCommand
	double linear_x
	double linear_y
	double angular_z

CommandFeedback
	map<FrameId, VelocityCommand> commands
```

# Enums

```cpp
Label
	EMPTY
	FIELD
    MR_STABS_MK1
	MR_STABS_MK2
	MRS_BUFF_MK1
	MRS_BUFF_MK2
	HOUSE_BOT
	OPPONENT
	MRS_BUFF_MK3

KeypointLabel
	EMPTY
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
	OPPONENT_FRONT
	OPPONENT_BACK
	MRS_BUFF_MK3_FRONT
	MRS_BUFF_MK3_BACK

Group
	OURS
	NEUTRAL
	THEIRS

FrameId
	EMPTY
	VISUAL_ODOMETRY
	CAMERA_WORLD
	CAMERA
	OUR_ROBOT_1
	OUR_ROBOT_2
	THEIR_ROBOT_1
	THEIR_ROBOT_2
	THEIR_ROBOT_3
	NEUTRAL_ROBOT_1
	NEUTRAL_ROBOT_2
	FIELD

DepthMode
	ZED_NONE
	ZED_PERFORMANCE
	ZED_QUALITY
	ZED_ULTRA
	ZED_NEURAL_LIGHT
	ZED_NEURAL
	ZED_NEURAL_PLUS

DeepLabModelType
	DeepLabV3
	DeepLabV3Plus

Resolution
	RES_3856x2180
	RES_3800x1800
	RES_2208x1242
	RES_1920x1536
	RES_1920x1080
	RES_1920x1200
	RES_1280x720
	RES_960x600
	RES_672x376
```
