// Drop into main.cpp

// Example usage in auton:
// distanceSensorCorrection(DistanceSensorId::FRONT); // Resets along bot's forward axis
// distanceSensorCorrection({DistanceSensorId::RIGHT}); // Resets along bot's horizontal axis

// Recommended that bot is either:
// - Standing still for about 20ms
// - Traveling parallel to the wall it is being corrected against

// Optional distance sensors for pose correction.
// Set the USE_* flags and ports for each installed sensor.

// 0 = off, 1 = enabled
#define USE_FRONT_DISTANCE_SENSOR 0
#define USE_RIGHT_DISTANCE_SENSOR 1
#define USE_BACK_DISTANCE_SENSOR 0
#define USE_LEFT_DISTANCE_SENSOR 1

constexpr std::uint8_t FRONT_DISTANCE_SENSOR_PORT = 0;
constexpr std::uint8_t RIGHT_DISTANCE_SENSOR_PORT = 7;
constexpr std::uint8_t BACK_DISTANCE_SENSOR_PORT = 0;
constexpr std::uint8_t LEFT_DISTANCE_SENSOR_PORT = 4;

// Offset from sensor to tracking center along that sensor's beam direction (inches).
constexpr float FRONT_DISTANCE_TO_TRACK_CENTER_IN = 0.0f;
constexpr float RIGHT_DISTANCE_TO_TRACK_CENTER_IN = 5.75f;
constexpr float BACK_DISTANCE_TO_TRACK_CENTER_IN = 0.0f;
constexpr float LEFT_DISTANCE_TO_TRACK_CENTER_IN = 5.75f;

#if USE_FRONT_DISTANCE_SENSOR
pros::Distance front_distance_sensor(FRONT_DISTANCE_SENSOR_PORT);
#endif
#if USE_RIGHT_DISTANCE_SENSOR
pros::Distance right_distance_sensor(RIGHT_DISTANCE_SENSOR_PORT);
#endif
#if USE_BACK_DISTANCE_SENSOR
pros::Distance back_distance_sensor(BACK_DISTANCE_SENSOR_PORT);
#endif
#if USE_LEFT_DISTANCE_SENSOR
pros::Distance left_distance_sensor(LEFT_DISTANCE_SENSOR_PORT);
#endif

enum class DistanceSensorId : std::size_t {
    FRONT = 0,
    RIGHT = 1,
    BACK = 2,
    LEFT = 3
};

struct DistanceResetProfile {
    const char* name;
    pros::Distance* sensor;
    float heading_offset_deg; // sensor heading relative to robot heading
    float center_offset_in;   // sensor->tracking-center offset along beam
    float min_valid_in;
    float max_valid_in;
};

struct WallInference {
    bool corrects_x;
    bool positive_wall;
    float cardinal_heading_deg;
};

constexpr float FIELD_MIN_X = -72.0f;
constexpr float FIELD_MAX_X = 72.0f;
constexpr float FIELD_MIN_Y = -72.0f;
constexpr float FIELD_MAX_Y = 72.0f;
constexpr float MM_TO_IN = 1.0f / 25.4f;
constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
constexpr float DISTANCE_CARDINAL_HEADING_TOLERANCE_DEG = 35.0f;
constexpr float DISTANCE_PROJECTION_MIN_ERROR_DEG = 7.0f; // only apply cosine correction when notably off-cardinal
constexpr float DISTANCE_SET_POSE_MARGIN_IN = 0.25f;       // avoid hard-clamping exactly on walls.
constexpr std::size_t DISTANCE_SAMPLE_COUNT = 3;
constexpr std::uint32_t DISTANCE_SAMPLE_DELAY_MS = 4;

const std::array<DistanceResetProfile, 4> DISTANCE_RESET_PROFILES = {{
    {
        "Front",
#if USE_FRONT_DISTANCE_SENSOR
        &front_distance_sensor,
#else
        nullptr,
#endif
        0.0f,
        FRONT_DISTANCE_TO_TRACK_CENTER_IN,
        2.0f,
        70.0f
    },
    {
        "Right",
#if USE_RIGHT_DISTANCE_SENSOR
        &right_distance_sensor,
#else
        nullptr,
#endif
        90.0f,
        RIGHT_DISTANCE_TO_TRACK_CENTER_IN,
        2.0f,
        70.0f
    },
    {
        "Back",
#if USE_BACK_DISTANCE_SENSOR
        &back_distance_sensor,
#else
        nullptr,
#endif
        180.0f,
        BACK_DISTANCE_TO_TRACK_CENTER_IN,
        2.0f,
        70.0f
    },
    {
        "Left",
#if USE_LEFT_DISTANCE_SENSOR
        &left_distance_sensor,
#else
        nullptr,
#endif
        -90.0f,
        LEFT_DISTANCE_TO_TRACK_CENTER_IN,
        2.0f,
        70.0f
    }
}};

std::size_t active_distance_sensor_index = static_cast<std::size_t>(DistanceSensorId::FRONT);

float normalize_degrees(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

bool infer_wall_from_heading(const float global_heading_deg, WallInference* wall) {
    if (wall == nullptr) {
        return false;
    }

    // Heading convention: 0 deg = +Y, positive angles rotate clockwise.
    static constexpr std::array<float, 4> CARDINAL_HEADINGS = {0.0f, 90.0f, 180.0f, -90.0f};

    const float normalized_heading = normalize_degrees(global_heading_deg);
    float min_error_deg = 1000.0f;
    float best_cardinal_deg = 0.0f;

    for (const float cardinal_deg : CARDINAL_HEADINGS) {
        const float error_deg = std::fabs(normalize_degrees(normalized_heading - cardinal_deg));
        if (error_deg < min_error_deg) {
            min_error_deg = error_deg;
            best_cardinal_deg = cardinal_deg;
        }
    }

    if (min_error_deg > DISTANCE_CARDINAL_HEADING_TOLERANCE_DEG) {
        return false;
    }

    if (best_cardinal_deg == 90.0f) {
        *wall = {true, true, best_cardinal_deg}; // +X wall
        return true;
    }
    if (best_cardinal_deg == -90.0f) {
        *wall = {true, false, best_cardinal_deg}; // -X wall
        return true;
    }
    if (best_cardinal_deg == 180.0f) {
        *wall = {false, false, best_cardinal_deg}; // -Y wall
        return true;
    }

    *wall = {false, true, best_cardinal_deg}; // 0 deg -> +Y wall
    return true;
}

bool set_active_distance_sensor(const DistanceSensorId sensor_id) {
    const std::size_t index = static_cast<std::size_t>(sensor_id);
    if (index >= DISTANCE_RESET_PROFILES.size()) {
        return false;
    }
    if (DISTANCE_RESET_PROFILES[index].sensor == nullptr) {
        return false;
    }
    active_distance_sensor_index = index;
    return true;
}

bool set_active_distance_sensor(const std::size_t sensor_index) {
    if (sensor_index >= DISTANCE_RESET_PROFILES.size()) {
        return false;
    }
    if (DISTANCE_RESET_PROFILES[sensor_index].sensor == nullptr) {
        return false;
    }
    active_distance_sensor_index = sensor_index;
    return true;
}

bool read_distance_inches(const DistanceResetProfile& profile, float* distance_in, std::int32_t* distance_mm) {
    if (profile.sensor == nullptr || distance_in == nullptr || distance_mm == nullptr) {
        return false;
    }

    std::array<std::int32_t, DISTANCE_SAMPLE_COUNT> valid_mm = {};
    std::size_t valid_count = 0;

    for (std::size_t i = 0; i < DISTANCE_SAMPLE_COUNT; i++) {
        const std::int32_t mm = profile.sensor->get();
        if (mm > 0 && mm != PROS_ERR && mm != 9999) {
            const float in = static_cast<float>(mm) * MM_TO_IN;
            if (in >= profile.min_valid_in && in <= profile.max_valid_in) {
                valid_mm[valid_count++] = mm;
            }
        }

        if (i + 1 < DISTANCE_SAMPLE_COUNT) {
            pros::delay(DISTANCE_SAMPLE_DELAY_MS);
        }
    }

    if (valid_count != DISTANCE_SAMPLE_COUNT) {
        return false;
    }

    std::sort(valid_mm.begin(), valid_mm.end());
    const std::int32_t median_mm = valid_mm[DISTANCE_SAMPLE_COUNT / 2];
    *distance_mm = median_mm;
    *distance_in = static_cast<float>(median_mm) * MM_TO_IN;
    return true;
}

bool reset_pose_axis_from_distance_profile(const DistanceResetProfile& profile) {
    if (profile.sensor == nullptr) {
        return false;
    }

    std::int32_t distance_mm = 0;
    float distance_in = 0.0f;
    if (!read_distance_inches(profile, &distance_in, &distance_mm)) {
        return false;
    }

    lemlib::Pose pose = chassis.getPose();
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        return false;
    }

    const float sensor_global_heading = pose.theta + profile.heading_offset_deg;
    WallInference wall = {};
    if (!infer_wall_from_heading(sensor_global_heading, &wall)) {
        return false;
    }

    // Convert beam distance to wall-normal distance when off-cardinal enough to matter.
    const float heading_error_deg = normalize_degrees(sensor_global_heading - wall.cardinal_heading_deg);
    float projection_scale = 1.0f;
    if (std::fabs(heading_error_deg) > DISTANCE_PROJECTION_MIN_ERROR_DEG) {
        projection_scale = std::cos(heading_error_deg * DEG_TO_RAD);
        if (projection_scale <= 0.0f) {
            return false;
        }
    }

    const float center_to_wall_in = (distance_in + profile.center_offset_in) * projection_scale;
    if (center_to_wall_in < 0.0f || center_to_wall_in > (FIELD_MAX_X - FIELD_MIN_X)) {
        return false;
    }

    if (wall.corrects_x) {
        const float corrected_x = wall.positive_wall ? (FIELD_MAX_X - center_to_wall_in)
                                                     : (FIELD_MIN_X + center_to_wall_in);
        pose.x = std::clamp(corrected_x, FIELD_MIN_X + DISTANCE_SET_POSE_MARGIN_IN, FIELD_MAX_X - DISTANCE_SET_POSE_MARGIN_IN);
    } else {
        const float corrected_y = wall.positive_wall ? (FIELD_MAX_Y - center_to_wall_in)
                                                     : (FIELD_MIN_Y + center_to_wall_in);
        pose.y = std::clamp(corrected_y, FIELD_MIN_Y + DISTANCE_SET_POSE_MARGIN_IN, FIELD_MAX_Y - DISTANCE_SET_POSE_MARGIN_IN);
    }

    chassis.setPose(pose.x, pose.y, pose.theta);

    if (DEBUG_MODE && pros::lcd::is_initialized()) {
        pros::lcd::print(6, "DRESET %s %4dmm H:%4.0f", profile.name, static_cast<int>(distance_mm), wall.cardinal_heading_deg);
        pros::lcd::print(7, "POSE  X:%6.2f Y:%6.2f", pose.x, pose.y);
    }
    return true;
}

// Command: choose a sensor profile and correct one pose axis from wall distance.
bool reset_pose_axis_from_distance(const DistanceSensorId sensor_id) {
    const std::size_t index = static_cast<std::size_t>(sensor_id);
    if (index >= DISTANCE_RESET_PROFILES.size()) {
        return false;
    }
    return reset_pose_axis_from_distance_profile(DISTANCE_RESET_PROFILES[index]);
}

bool reset_pose_axis_from_distance(const std::size_t sensor_index) {
    if (sensor_index >= DISTANCE_RESET_PROFILES.size()) {
        return false;
    }
    return reset_pose_axis_from_distance_profile(DISTANCE_RESET_PROFILES[sensor_index]);
}

bool reset_pose_axis_from_active_distance() {
    return reset_pose_axis_from_distance(active_distance_sensor_index);
}

// select active sensor + apply correction in one call.
bool distanceSensorCorrection(const DistanceSensorId sensor_id) {
    if (!set_active_distance_sensor(sensor_id)) {
        return false;
    }
    return reset_pose_axis_from_active_distance();
}

bool distanceSensorCorrection(const std::size_t sensor_index) {
    if (!set_active_distance_sensor(sensor_index)) {
        return false;
    }
    return reset_pose_axis_from_active_distance();
}
