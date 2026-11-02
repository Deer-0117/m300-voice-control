#ifndef TARGET_CHASE_H
#define TARGET_CHASE_H

#include <stdbool.h>
#include <stdint.h>
#include "target_chase_delayed_kf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TARGET_CHASE_MODE_MANUAL = 0,
    TARGET_CHASE_MODE_WAITING_AUTHORITY,
    TARGET_CHASE_MODE_ACTIVE,
    TARGET_CHASE_MODE_HOLD,
    TARGET_CHASE_MODE_TARGET_LOST,
} E_TargetChaseMode;

typedef enum {
    /*
     * Forward-looking camera, airborne target, no depth/range input.
     * The controller uses image bearing only: keep the target inside a
     * box-size-adaptive center window while approaching along body-forward.
     */
    TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY = 0,

    /*
     * Nadir camera and ground target. Pixel offsets are projected onto the
     * ground plane by using current aircraft height.
     */
    TARGET_CHASE_GEOMETRY_GROUND_NADIR = 1,
} E_TargetChaseGeometry;

typedef struct {
    E_TargetChaseGeometry geometry;

    float image_width_px;
    float image_height_px;
    float fx_px;
    float fy_px;
    float cx_px;
    float cy_px;

    /*
     * AIRBORNE_BEARING_ONLY:
     * pixel x -> yaw bearing, pixel y -> vertical bearing.
     * Defaults assume a forward camera with image x right and image y down.
     *
     * GROUND_NADIR:
     * pixel x -> body-right target offset, pixel y -> body-forward target offset.
     *
     * Flip these signs if your image axes are mounted differently.
     */
    float pixel_x_to_yaw_sign;
    float pixel_y_to_up_sign;
    float pixel_x_to_body_right_sign;
    float pixel_y_to_body_forward_sign;

    float desired_height_m;       /* <= 0 means hold height at handoff time. */
    float min_height_m;
    float max_height_m;
    float target_timeout_ms;
    float stop_radius_m;
    float yaw_deadband_deg;
    float bearing_deadband_deg;
    float vertical_bearing_deadband_deg;
    float max_approach_bearing_deg; /* Legacy/telemetry threshold. */

    /*
     * Extra center deadband derived from detector box angular size:
     * effective deadband = base deadband + box_half_diagonal_bearing * scale.
     * Larger YOLO boxes therefore tolerate a wider center region.
     */
    float box_dynamic_deadband_scale;
    float approach_speed_mps;      /* Forward speed for airborne bearing-only chase. */
    float min_approach_speed_mps;  /* Minimum forward speed when the target box is large. */
    float box_angle_small_deg;     /* Box half-diagonal angle that still uses full speed. */
    float box_angle_large_deg;     /* Box half-diagonal angle that uses minimum speed. */
    float gimbal_mount_yaw_offset_deg; /* Home optical-axis yaw, subtracted so 0 is body-forward. */
    float gimbal_alignment_deadband_deg; /* Forward waits until optical-axis yaw is nearly body-forward. */
    float gimbal_recenter_enter_deg; /* Body starts helping after gimbal yaw crosses this threshold. */
    float gimbal_recenter_exit_deg;  /* Body stops helping only after returning inside this threshold. */
    float prediction_horizon_s;      /* Detector/actuator delay compensation horizon. */
    int prediction_mode; /* 0=existing world, 1=original pixel, 2=near hybrid, 3=aligned delayed KF. */
    bool prediction_adaptive;         /* Quality, age, edge and near-size lead limits. */
    float prediction_residual_min_deg;
    float prediction_residual_box_scale;
    float motion_filter_alpha;       /* Alpha-beta observation correction gain. */
    float motion_filter_beta;        /* Alpha-beta angular-rate correction gain. */
    float mpc_horizon_s;
    float mpc_step_s;
    float mpc_max_accel_mps2;
    float mpc_weight_image;
    float mpc_weight_gimbal_center;
    float mpc_weight_distance;
    float mpc_weight_acceleration;
    float mpc_weight_yaw_rate;
    float desired_box_angle_deg; /* Angular-size distance proxy for the approach objective. */

    float max_horizontal_speed_mps;
    float max_vertical_speed_mps;
    float max_vertical_excursion_m; /* Max climb/descent from the first valid in-air handoff height. */
    float max_yaw_rate_degps;
    float max_tilt_deg;
    float max_angular_rate_radps;

    float kp_position;
    float kd_velocity;
    float kp_height;
    float kd_vertical_velocity;
    float kp_yaw_rate;
    float kp_vertical_bearing;    /* Legacy tuning field; airborne line-follow no longer uses angle P gain. */
} T_TargetChaseConfig;

typedef struct {
    float altitude_agl_m;
    float velocity_neu_mps[3];    /* north, east, up */
    float attitude_deg[3];        /* roll, pitch, yaw */
    float angular_rate_radps[3];
    bool in_air;
    bool velocity_healthy;
    uint64_t timestamp_ms;
} T_TargetChaseAircraftState;

typedef struct {
    float u_px;
    float v_px;
    float box_width_px;
    float box_height_px;
    bool has_box;
    bool valid;
    uint64_t timestamp_ms;
} T_TargetChaseTargetPixel;

typedef struct {
    float x_mps;                  /* body-forward velocity command */
    float y_mps;                  /* body-right velocity command */
    float z_mps;                  /* upward velocity command */
    float yaw_rate_degps;

    float target_offset_forward_m;
    float target_offset_right_m;
    float target_distance_m;
    float target_bearing_yaw_deg;
    float target_bearing_vertical_deg;

    /* Runtime thresholds after applying YOLO box-size dynamic expansion. */
    float effective_yaw_deadband_deg;
    float effective_bearing_deadband_deg;
    float effective_approach_bearing_deg;
    float target_box_width_px;
    float target_box_height_px;
    bool reached;
    bool should_send;
} T_TargetChaseCommand;

typedef struct {
    float horizontal_fov_deg;
    float vertical_fov_deg;
    float gimbal_yaw_limit_deg;
    float gimbal_pitch_min_deg;
    float gimbal_pitch_max_deg;
    float sector_width_deg;
    float vertical_coverage_deg;
    float requested_overlap_deg;
    float actual_overlap_deg;
    unsigned int sector_count;
    float body_yaw_step_deg;
} T_TargetChaseSearchCoverage;

typedef enum {
    TARGET_KF_DISABLED = 0,
    TARGET_KF_NO_CALIBRATION,
    TARGET_KF_NO_POSE,
    TARGET_KF_WARMUP,
    TARGET_KF_ACTIVE,
    TARGET_KF_REJECTED,
    TARGET_KF_STALE,
    TARGET_KF_UNCERTAIN,
    TARGET_KF_RESET,
} E_TargetChaseKfReason;

typedef struct {
    T_TargetChaseConfig config;
    T_TargetChaseAircraftState aircraft;
    T_TargetChaseTargetPixel target;
    /* Camera optical-axis angles relative to the aircraft body. */
    float camera_boresight_yaw_deg;
    float camera_boresight_vertical_deg;
    E_TargetChaseMode mode;
    bool manual_trigger_enabled;
    bool has_authority;
    bool desired_height_initialized;
    float desired_height_m;
    uint64_t pixel_timestamp_ms;
    float pixel_u, pixel_v, pixel_du, pixel_dv;
    unsigned int pixel_samples;
    float hybrid_weight, hybrid_horizon_s, hybrid_yaw_rate;
    bool motion_estimate_initialized;
    uint64_t motion_estimate_timestamp_ms;
    float filtered_u_px;
    float filtered_v_px;
    float filtered_u_rate_pxps;
    float filtered_v_rate_pxps;
    float filtered_box_angle_deg;
    float filtered_box_angle_rate_degps;
    /* Inertial line-of-sight estimate, measured at the image timestamp. */
    bool observation_pose_valid;
    uint64_t observation_pose_timestamp_ms;
    float observation_camera_yaw_deg;
    float observation_camera_pitch_deg;
    bool motion_world_valid;
    float filtered_world_yaw_deg;
    float filtered_world_pitch_deg;
    float filtered_world_yaw_rate_degps;
    float filtered_world_pitch_rate_degps;
    float prediction_residual_deg;
    float prediction_quality;
    unsigned int prediction_recovery_frames;
    bool gimbal_recenter_active;
    float last_planned_forward_mps;
    float last_planned_yaw_rate_degps;
    /* Independent from the compatibility alpha-beta estimators. */
    T_TargetDelayedKf delayed_kf;
    E_TargetChaseKfReason kf_reason;
    bool kf_active;
    float kf_variance_deg2;
    uint64_t kf_measurement_ms;
    uint64_t kf_epoch_ms;
    uint64_t kf_valid_until_ms;
    bool kf_camera_pose_valid;
    uint64_t kf_camera_pose_ms;
    float kf_camera_yaw_deg, kf_camera_pitch_deg;
} T_TargetChaseController;

typedef enum {
    TARGET_PREDICTION_FIXED = 0,
    TARGET_PREDICTION_NORMAL,
    TARGET_PREDICTION_RECOVERING,
    TARGET_PREDICTION_AGED,
    TARGET_PREDICTION_EDGE,
    TARGET_PREDICTION_NO_POSE,
    TARGET_PREDICTION_INVALID_TIME,
    TARGET_PREDICTION_NEAR_LIMIT,
} E_TargetPredictionReason;

typedef struct {
    float world_yaw_deg;
    float world_pitch_deg;
    float horizon_s;          /* Effective extrapolation after quality scaling. */
    float rate_scale;         /* Apply once to MPC/gimbal velocity feed-forward. */
    float age_horizon_s;      /* Effective compensation for elapsed observation time. */
    float extra_horizon_s;    /* Effective future lead beyond the control tick. */
    E_TargetPredictionReason reason;
} T_TargetChasePrediction;

bool TargetChase_PredictWorld(const T_TargetChaseController *controller,
                              uint64_t now_ms, T_TargetChasePrediction *prediction);
float TargetChase_PredictionHorizon(const T_TargetChaseController *controller,
                                    uint64_t now_ms);
/* Apply after final vertical limits/slew too, so downstream shaping cannot
 * restore forward motion unsupported by the allowed climb/descent speed. */
bool TargetChase_LimitForwardForAlignment(const T_TargetChaseController *controller,
                                         T_TargetChaseCommand *command);
bool TargetChase_LimitForwardForVertical(const T_TargetChaseConfig *config,
                                        T_TargetChaseCommand *command);

T_TargetChaseConfig TargetChase_DefaultConfig(float image_width_px,
                                              float image_height_px,
                                              float horizontal_fov_deg,
                                              float vertical_fov_deg);

/*
 * Convert camera FOV and gimbal travel into a cyclic 360-degree search plan.
 * The returned body step is 360 / sector_count and never exceeds the
 * non-overlapped width of one gimbal sector.
 */
bool TargetChase_ComputeSearchCoverage(float horizontal_fov_deg,
                                      float vertical_fov_deg,
                                      float gimbal_yaw_limit_deg,
                                      float gimbal_pitch_min_deg,
                                      float gimbal_pitch_max_deg,
                                      float requested_overlap_deg,
                                      T_TargetChaseSearchCoverage *coverage);

/*
 * Update image geometry when the detector input resolution changes. The
 * normalized focal lengths are preserved, so the configured field of view
 * remains unchanged while cx/cy and fx/fy follow the live frame dimensions.
 */
bool TargetChase_UpdateImageGeometry(T_TargetChaseController *controller,
                                     float image_width_px,
                                     float image_height_px);

void TargetChase_Init(T_TargetChaseController *controller,
                      const T_TargetChaseConfig *config);

void TargetChase_UpdateAircraftState(T_TargetChaseController *controller,
                                     const T_TargetChaseAircraftState *state);

/*
 * Set the camera optical-axis direction relative to the aircraft body. A
 * fixed forward camera uses (0, 0). A movable gimbal should refresh these
 * values from gimbal attitude before each target update so aircraft steering
 * remains correct even after the target has been centered in the image.
 */
void TargetChase_SetCameraBoresight(T_TargetChaseController *controller,
                                    float yaw_deg,
                                    float vertical_deg);

/* Supply the camera's inertial yaw/pitch at the frame timestamp before the
 * corresponding target update. A missing pose leaves prediction disabled for
 * that observation, avoiding camera rotation being mistaken for target motion. */
void TargetChase_SetObservationCameraPose(T_TargetChaseController *controller,
                                          float world_yaw_deg,
                                          float world_pitch_deg,
                                          uint64_t timestamp_ms);

void TargetChase_UpdateTargetPixel(T_TargetChaseController *controller,
                                   float u_px,
                                   float v_px,
                                   uint64_t timestamp_ms);

/*
 * Update target from a detector box. Coordinates may be provided as
 * left-bottom and right-top in image pixels; the implementation normalizes
 * min/max internally, so image y-axis direction does not matter.
 */
void TargetChase_UpdateTargetBox(T_TargetChaseController *controller,
                                 float left_bottom_u_px,
                                 float left_bottom_v_px,
                                 float right_top_u_px,
                                 float right_top_v_px,
                                 uint64_t timestamp_ms);

bool TargetChase_PredictHybrid(T_TargetChaseController *controller, uint64_t now_ms,
    float camera_world_yaw, float camera_world_pitch,
    float *yaw, float *pitch, float *yaw_rate, float *pitch_rate);

/* Only the calibrated runtime adapter may supply these world bearings.
 * This does not replace the latest image target or refresh its timeout. */
bool TargetChase_ObserveAlignedBearing(T_TargetChaseController *controller,
    float world_yaw_deg, float world_pitch_deg,
    uint64_t measurement_ms, uint64_t received_ms);
void TargetChase_ResetDelayedPrediction(T_TargetChaseController *controller,
    E_TargetChaseKfReason reason);

void TargetChase_ClearTarget(T_TargetChaseController *controller);

void TargetChase_SetManualTrigger(T_TargetChaseController *controller,
                                  bool enabled,
                                  uint64_t now_ms);

bool TargetChase_IsManualTriggerEnabled(const T_TargetChaseController *controller);

void TargetChase_SetAuthority(T_TargetChaseController *controller,
                              bool has_authority,
                              uint64_t now_ms);

bool TargetChase_ShouldObtainAuthority(const T_TargetChaseController *controller,
                                       uint64_t now_ms);

bool TargetChase_ShouldReleaseAuthority(const T_TargetChaseController *controller,
                                        uint64_t now_ms);

bool TargetChase_Step(T_TargetChaseController *controller,
                      uint64_t now_ms,
                      T_TargetChaseCommand *command);

E_TargetChaseMode TargetChase_GetMode(const T_TargetChaseController *controller);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_CHASE_H */
