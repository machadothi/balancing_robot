/**
 * @file test_filters.c
 * @brief Attitude filters, through AttitudeFilter_t: every filter must pass all
 */

#include "filter/filter.h"
#include "unit.h"

#define DT 0.01f

static KalmanFilter_t kalman;
static ComplementaryFilter_t complementary;

static void reset_filters(void) {
    kalman_init(&kalman);
    complementary_init(&complementary);
}

static void test_seed_starts_at_the_measurement(AttitudeFilter_t f) {
    f.seed(f.state, 90.0f);
    CHECK_NEAR(f.update(f.state, 0.0f, 90.0f, DT), 90.0, 1e-3);
}

static void test_converges_to_accelerometer(AttitudeFilter_t f) {
    float angle = 0.0f;
    for (int i = 0; i < 1000; i++) {
        angle = f.update(f.state, 0.0f, 10.0f, DT);
    }
    CHECK_NEAR(angle, 10.0, 0.1);
}

/* 20 deg/s rotation for 2 s with gyro and accelerometer agreeing */
static void test_tracks_rotation(AttitudeFilter_t f) {
    float truth = 0.0f;
    float angle = 0.0f;

    f.seed(f.state, truth);
    for (int i = 0; i < 200; i++) {
        truth += 20.0f * DT;
        angle = f.update(f.state, 20.0f, truth, DT);
    }
    CHECK_NEAR(angle, truth, 0.5);
}

/* The gyro carries the estimate through a short accelerometer disturbance */
static void test_attenuates_accelerometer_spike(AttitudeFilter_t f) {
    for (int i = 0; i < 1000; i++) {
        (void)f.update(f.state, 0.0f, 0.0f, DT);
    }
    float angle = f.update(f.state, 0.0f, 30.0f, DT);
    CHECK(fabs(angle) < 15.0);
}

int main(void) {
    AttitudeFilter_t filters[] = {
        kalman_filter_interface(&kalman),
        complementary_filter_interface(&complementary),
    };

    for (size_t i = 0; i < sizeof(filters) / sizeof(filters[0]); i++) {
        int before = unit_failures;

        reset_filters();
        test_seed_starts_at_the_measurement(filters[i]);
        reset_filters();
        test_converges_to_accelerometer(filters[i]);
        reset_filters();
        test_tracks_rotation(filters[i]);
        reset_filters();
        test_attenuates_accelerometer_spike(filters[i]);

        if (unit_failures != before) {
            printf("  ^ failures above are for the %s filter\n", filters[i].name);
        }
    }
    return UNIT_RESULT();
}
