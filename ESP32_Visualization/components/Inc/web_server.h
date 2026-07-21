#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>

/* Start HTTP server and WebSocket endpoint. */
void web_server_init(void);

/* Send sound direction to all WS clients. azimuth [0,360): 0=+X fwd, 90=+Y left;
 * polar = elevation [-90,+90], +90=up (+Z), 0=horizon. */
void web_server_send_data(float azimuth, float polar, uint8_t strength);

/* Send board orientation (IMU) to all WS clients so the view rotates the plane.
 * roll = about X [-180..180], pitch = about Y [-90..90]; yaw unused. */
void web_server_send_orientation(float roll, float pitch);

#endif