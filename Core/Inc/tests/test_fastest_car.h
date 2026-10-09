#ifndef TEST_FASTEST_CAR_H
#define TEST_FASTEST_CAR_H

/*
 * FASTEST CAR TASK CONFIGURATION
 * ==============================
 * The briefing requires the car to obey the arrow on each of the two goal
 * obstacles. Until camera recognition is connected, configure the two arrow
 * results here. These are the only two direction selections normally changed.
 */
#define FASTEST_CAR_TURN_LEFT                    (-1)
#define FASTEST_CAR_TURN_RIGHT                   (1)
#define FASTEST_CAR_FIRST_OBSTACLE_TURN          FASTEST_CAR_TURN_LEFT
#define FASTEST_CAR_SECOND_OBSTACLE_TURN         FASTEST_CAR_TURN_RIGHT

#define FASTEST_CAR_START_DELAY_MS               3000U
#define FASTEST_CAR_MAX_RUN_MS                   175000U

#define FASTEST_CAR_FRONT_STOP_CM                32U
#define FASTEST_CAR_FRONT_EMERGENCY_CM           16U
#define FASTEST_CAR_FRONT_SLOW_CM                80U

#define FASTEST_CAR_SIDE_TARGET_CM               22U
#define FASTEST_CAR_SIDE_DEADBAND_CM              3U
#define FASTEST_CAR_SIDE_MIN_CM                  11U
#define FASTEST_CAR_EDGE_LOST_CM                 45U
#define FASTEST_CAR_LEFT_SAFE_RADIUS_CM          20

/* Fixed steering strengths used after the ultrasonic sensor sees a goal.
 * Positive magnitudes are converted to the correct servo direction in code.
 * The left command is also clamped to the configured >=20 cm safe radius. */
#define FASTEST_CAR_LEFT_TURN_CORRECTION_US      650
#define FASTEST_CAR_RIGHT_TURN_CORRECTION_US     650
#define FASTEST_CAR_FOLLOW_MAX_CORRECTION_US     280
#define FASTEST_CAR_STEER_SLEW_US_PER_CYCLE       24

#define FASTEST_CAR_CRUISE_PWM                   3300U
#define FASTEST_CAR_APPROACH_PWM                 2550U
#define FASTEST_CAR_TURN_PWM                     2600U
#define FASTEST_CAR_REVERSE_PWM                  2300U

/* The entry turn must clear both a minimum yaw and the ultrasonic front zone.
 * Timing remains as a fail-safe if a gyro sample is temporarily unavailable. */
#define FASTEST_CAR_ENTRY_TURN_ANGLE_DEG         55.0f
#define FASTEST_CAR_ENTRY_TURN_MIN_MS            950U
#define FASTEST_CAR_ENTRY_TURN_MAX_MS            2600U
#define FASTEST_CAR_FOLLOW_MIN_MS                1600U
#define FASTEST_CAR_FOLLOW_MAX_MS                9000U
#define FASTEST_CAR_PRELOCK_TURN_MAX_MS          3200U
#define FASTEST_CAR_EXIT_TURN_MS                 900U
#define FASTEST_CAR_RECENTER_MIN_MS              1000U
#define FASTEST_CAR_RECENTER_MAX_MS              3200U
#define FASTEST_CAR_RECENTER_YAW_TOL_DEG         8.0f
#define FASTEST_CAR_RETURN_ARC_MS                1700U
#define FASTEST_CAR_RETURN_DRIVE_MS              5000U
#define FASTEST_CAR_ARROW_SETTLE_MS              350U
#define FASTEST_CAR_EMERGENCY_REVERSE_MS         220U

#if FASTEST_CAR_LEFT_SAFE_RADIUS_CM < 20
#error "Fastest Car left turn radius must be at least 20 cm"
#endif

#if FASTEST_CAR_LEFT_TURN_CORRECTION_US <= 0 || \
    FASTEST_CAR_RIGHT_TURN_CORRECTION_US <= 0
#error "Fastest Car turn correction magnitudes must be positive"
#endif

#if ((FASTEST_CAR_FIRST_OBSTACLE_TURN != FASTEST_CAR_TURN_LEFT) && \
     (FASTEST_CAR_FIRST_OBSTACLE_TURN != FASTEST_CAR_TURN_RIGHT)) || \
    ((FASTEST_CAR_SECOND_OBSTACLE_TURN != FASTEST_CAR_TURN_LEFT) && \
     (FASTEST_CAR_SECOND_OBSTACLE_TURN != FASTEST_CAR_TURN_RIGHT))
#error "Fastest Car obstacle turns must be FASTEST_CAR_TURN_LEFT or FASTEST_CAR_TURN_RIGHT"
#endif

void Test_FastestCar(void);

#endif /* TEST_FASTEST_CAR_H */
