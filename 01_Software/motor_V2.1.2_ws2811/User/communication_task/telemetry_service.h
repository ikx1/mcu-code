#ifndef __TELEMETRY_SERVICE_H__
#define __TELEMETRY_SERVICE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t left_wheel_angular_mrad_s;
    int32_t right_wheel_angular_mrad_s;
    int32_t z_lift_mm;
    uint16_t status_word;
    uint16_t health_word;
    uint16_t alarm_info;
    uint16_t batt_soc_x100;
    uint16_t bucket_volume_ml;
    uint8_t bucket_volume_valid;
} telemetry_service_snapshot_t;

void telemetry_service_snapshot(telemetry_service_snapshot_t *out);
void telemetry_service_diag_maintenance(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_SERVICE_H__ */
