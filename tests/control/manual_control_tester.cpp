// manual_control_tester.cpp
// Standalone C++ MAVLink MANUAL_CONTROL sender, byte-matched to QGC v4.x packing:
//   target = FCU sysid
//   x = pitch * 1000,  y = roll * 1000,  z = thrust * 1000,  r = yaw * 1000
// (QGC Vehicle::sendJoystickDataThreadSafe maps these four fields into the
//  MANUAL_CONTROL message in exactly this order, axes scaled by 1000.)
//
// ArduSub expects z in [0,1000] (legacy): z=500 -> RC throttle 1500 = neutral.
//
// Build:
//   g++ -std=c++17 -O2 -I/opt/ros/humble/include tests/control/manual_control_tester.cpp -o /tmp/mc_tester
// Run (via mavlink-router, MAVROS/QGC stopped):
//   /tmp/mc_tester --sweep
//   /tmp/mc_tester --interactive
#include "mavlink/v2.0/ardupilotmega/mavlink.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <thread>

// ---------------------------------------------------------------------------
namespace {

int g_fd = -1;
sockaddr_in g_router{};
sockaddr_in g_peer{};            // learned source of inbound datagrams (mirrors
bool g_peer_learned = false;     // pymavlink udpin: reply to the router's source,
                                 // NOT to 127.0.0.1:14555 itself)
std::atomic<bool> g_run{true};

// --- MAVLink send helpers ---------------------------------------------------
void mav_send(const mavlink_message_t &msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (getenv("MC_DUMP") && msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
        fprintf(stderr, "MC z dump: ");
        for (uint16_t i = 0; i < len; i++) fprintf(stderr, "%02x ", buf[i]);
        fprintf(stderr, "\n");
        const uint8_t *p = reinterpret_cast<const uint8_t *>(_MAV_PAYLOAD(&msg));
        fprintf(stderr, "MC fields: x=%d y=%d z=%d r=%d buttons=%u target=%u (LE payload off 0/2/4/6/8/10)\n",
                *(const int16_t *)(p + 0), *(const int16_t *)(p + 2),
                *(const int16_t *)(p + 4), *(const int16_t *)(p + 6),
                *(const uint16_t *)(p + 8), p[10]);
        fprintf(stderr, "MC modern: target=%u x=%d y=%d z=%d r=%d buttons=%u (LE payload off 0/1/3/5/7/9)\n",
                p[0], *(const int16_t *)(p + 1), *(const int16_t *)(p + 3),
                *(const int16_t *)(p + 5), *(const int16_t *)(p + 7),
                *(const uint16_t *)(p + 9));
    }
    const sockaddr *dest =
        g_peer_learned ? reinterpret_cast<sockaddr *>(&g_peer)
                       : reinterpret_cast<sockaddr *>(&g_router);
    if (g_peer_learned && g_peer.sin_port != g_router.sin_port) {
        sendto(g_fd, buf, len, 0, reinterpret_cast<sockaddr *>(&g_peer), sizeof(g_peer));
    } else {
        sendto(g_fd, buf, len, 0, reinterpret_cast<sockaddr *>(&g_router), sizeof(g_router));
    }
}

void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(255, 190, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    mav_send(msg);
}

void send_set_mode(uint8_t mode) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(255, 190, &msg, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode);
    mav_send(msg);
}

void send_arm_command(uint8_t arm) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 190, &msg, 1, 0,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  arm, 0, 0, 0, 0, 0, 0);
    mav_send(msg);
}

// QGC-mirror send path, hand-built MAVLink1 frame.
//
// We hand-assemble MANUAL_CONTROL (msgid 69) as a MAVLink1 frame to make the
// wire bytes EXACTLY match pymavlink's v1 emission, which ArduSub v4.7 has
// been verified to decode correctly:
//   STX=0xFE LEN=11 SEQ SYS=255 COMP=190 MSGID=69
//   payload (legacy order): [x(int16 LE)=pitch, y=roll, z=thrust, r=yaw,
//                            buttons(uint16)=0, target(uint8)=1]
//   CRC-16/MCRF4XX (poly 0x1021, init 0xFFFF, reflected) over
//   [LEN,SEQ,SYS,COMP,MSGID] + payload + crc_extra(243), low byte first.
// The CRC recipe reproduces pymavlink's z=500 frame byte-for-byte
// (fe 0b .. ff be 45 00 00 00 00 f4 01 00 00 00 00 01 69 1f).
uint8_t g_mc_seq = 0;

// CRC-16/MCRF4XX, bitwise reflected (poly 0x1021 -> reflected 0x8408).
static uint16_t mc_crc(const uint8_t *data, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = static_cast<uint16_t>((crc >> 1) ^ 0x8408);
            else         crc >>= 1;
        }
    }
    return crc;
}

void send_manual_control(int16_t pitch, int16_t roll, int16_t thrust, int16_t yaw) {
    uint8_t buf[24];
    buf[0] = 0xFE;                                    // STX (MAVLink1)
    buf[1] = 11;                                      // payload length
    buf[2] = g_mc_seq++;
    buf[3] = 255;                                     // sysid
    buf[4] = 190;                                     // compid
    buf[5] = MAVLINK_MSG_ID_MANUAL_CONTROL;           // 69
    uint8_t *pl = buf + 6;
    const int16_t axes[4] = { pitch, roll, thrust, yaw };
    for (int i = 0; i < 4; i++) {
        pl[i * 2 + 0] = static_cast<uint8_t>(axes[i] & 0xFF);
        pl[i * 2 + 1] = static_cast<uint8_t>(axes[i] >> 8);
    }
    pl[8] = 0;                  // buttons uint16 LE
    pl[9] = 0;
    pl[10] = 1;                 // target = FCU sysid (legacy: last byte)
    // CRC-16/MCRF4XX over [LEN,SEQ,SYS,COMP,MSGID] + payload + crc_extra(243).
    uint8_t crcbuf[17];
    memcpy(crcbuf, buf + 1, 16);
    crcbuf[16] = MAVLINK_MSG_ID_MANUAL_CONTROL_CRC;   // 243
    const uint16_t ck = mc_crc(crcbuf, sizeof(crcbuf));
    buf[17] = static_cast<uint8_t>(ck & 0xFF);
    buf[18] = static_cast<uint8_t>(ck >> 8);

    if (getenv("MC_DUMP")) {
        fprintf(stderr, "MC v1 wire: ");
        for (int i = 0; i < 19; i++) fprintf(stderr, "%02x ", buf[i]);
        fprintf(stderr, "\nMC v1 dec : target=%u x=%d y=%d z=%d r=%d buttons=%u "
                        "(LE payload off 0/2/4/6/8 + target off 10)\n",
                pl[10],
                *(const int16_t *)(pl + 0), *(const int16_t *)(pl + 2),
                *(const int16_t *)(pl + 4), *(const int16_t *)(pl + 6),
                *(const uint16_t *)(pl + 8));
    }
    const sockaddr *dest =
        g_peer_learned ? reinterpret_cast<sockaddr *>(&g_peer)
                       : reinterpret_cast<sockaddr *>(&g_router);
    if (g_peer_learned && g_peer.sin_port != g_router.sin_port) {
        sendto(g_fd, buf, 19, 0, reinterpret_cast<sockaddr *>(&g_peer), sizeof(g_peer));
    } else {
        sendto(g_fd, buf, 19, 0, reinterpret_cast<sockaddr *>(&g_router), sizeof(g_router));
    }
}

// --- RC_CHANNELS_OVERRIDE ------------------------------------------------
// msgid 70, crc_extra 124. Legacy 18-byte payload:
//   [target_system(1), target_component(1), chan1..chan8 (8x int16 LE)]
// This is the path QGC uses for ArduSub joystick, and the only one that
// rests AUX at a clean 1500/1500 at neutral (MANUAL_CONTROL rests ~1202).
uint8_t g_rc_seq = 0;

void send_rc_override(const uint16_t ch[8]) {
    uint8_t buf[28];
    buf[0] = 0xFE;                              // MAVLink1
    buf[1] = 18;                                // payload length
    buf[2] = g_rc_seq++;
    buf[3] = 255;                               // sysid
    buf[4] = 190;                               // compid
    buf[5] = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE; // 70
    uint8_t *pl = buf + 6;
    pl[0] = 1;                                  // target_system = FCU
    pl[1] = 0;                                  // target_component
    for (int i = 0; i < 8; i++) {
        pl[2 + i * 2 + 0] = static_cast<uint8_t>(ch[i] & 0xFF);
        pl[2 + i * 2 + 1] = static_cast<uint8_t>(ch[i] >> 8);
    }
    uint8_t crcbuf[19];
    memcpy(crcbuf, buf + 1, 18);
    crcbuf[18] = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC; // 124
    const uint16_t ck = mc_crc(crcbuf, sizeof(crcbuf));
    buf[24] = static_cast<uint8_t>(ck & 0xFF);
    buf[25] = static_cast<uint8_t>(ck >> 8);

    if (getenv("RC_DUMP")) {
        fprintf(stderr, "RC ch1..8 = ");
        for (int i = 0; i < 8; i++) fprintf(stderr, "%u ", ch[i]);
        fprintf(stderr, "| wire: ");
        for (int i = 0; i < 26; i++) fprintf(stderr, "%02x ", buf[i]);
        fprintf(stderr, "\n");
    }
    const sockaddr *dest =
        g_peer_learned ? reinterpret_cast<sockaddr *>(&g_peer)
                       : reinterpret_cast<sockaddr *>(&g_router);
    if (g_peer_learned && g_peer.sin_port != g_router.sin_port) {
        sendto(g_fd, buf, 26, 0, reinterpret_cast<sockaddr *>(&g_peer), sizeof(g_peer));
    } else {
        sendto(g_fd, buf, 26, 0, reinterpret_cast<sockaddr *>(&g_router), sizeof(g_router));
    }
}

// --- state ---------------------------------------------------------------
bool g_armed = false;
uint32_t g_mode = UINT32_MAX;
uint16_t g_servo[16] = {0};
bool g_servo_valid = false;
char g_last_ack[128] = "";

void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        g_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        g_mode = hb.custom_mode;
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&msg, &rc);
        if (getenv("RC_DIAG")) {
            printf("[RC_CH] ch1=%u ch2=%u ch3=%u ch4=%u\n",
                   rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw);
            fflush(stdout);
        }
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        snprintf(g_last_ack, sizeof(g_last_ack), "cmd=%u result=%u",
                 ack.command, ack.result);
        printf("\n  <<< COMMAND_ACK cmd=%u result=%u\n", ack.command, ack.result);
        fflush(stdout);
        break;
    }
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t s;
        mavlink_msg_servo_output_raw_decode(&msg, &s);
        g_servo[0]  = s.servo1_raw;  g_servo[1]  = s.servo2_raw;
        g_servo[2]  = s.servo3_raw;  g_servo[3]  = s.servo4_raw;
        g_servo[4]  = s.servo5_raw;  g_servo[5]  = s.servo6_raw;
        g_servo[6]  = s.servo7_raw;  g_servo[7]  = s.servo8_raw;
        g_servo[8]  = s.servo9_raw;  g_servo[9]  = s.servo10_raw;
        g_servo[10] = s.servo11_raw; g_servo[11] = s.servo12_raw;
        g_servo[12] = s.servo13_raw; g_servo[13] = s.servo14_raw;
        g_servo[14] = s.servo15_raw; g_servo[15] = s.servo16_raw;
        g_servo_valid = true;
        break;
    }
    default:
        break;
    }
}

void pump_until(bool (*cond)(), int timeout_ms) {
    const int step = 20;
    int waited = 0;
    while (g_run && !(cond && cond())) {
        fd_set r;
        FD_ZERO(&r);
        FD_SET(g_fd, &r);
        timeval tv{0, static_cast<suseconds_t>(step * 1000)};
        const int n = select(g_fd + 1, &r, nullptr, nullptr, &tv);
        if (n > 0) {
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            sockaddr_in src{};
            socklen_t srclen = sizeof(src);
            ssize_t rn = recvfrom(g_fd, buf, sizeof(buf), 0,
                                  reinterpret_cast<sockaddr *>(&src), &srclen);
            // Learn the peer's source address like pymavlink udpin does: the
            // router forwards the FCU stream from its own socket -- replying
            // there is what actually reaches the FCU. Skip packets that looped
            // back from our own 127.0.0.1:14555 bind.
            if (rn > 0 && !g_peer_learned && src.sin_port != htons(14555)) {
                g_peer = src;
                g_peer_learned = true;
            }
            if (rn > 0)
            for (ssize_t i = 0; i < rn; i++) {
                mavlink_message_t msg;
                mavlink_status_t st;
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) {
                    handle_message(msg);
                }
            }
        }
        waited += step;
        if (timeout_ms > 0 && waited >= timeout_ms) break;
    }
}

// --- command retries ----------------------------------------------------
// The router's shared 127.0.0.1:14555 peer means our socket shares the port;
// SO_REUSEPORT load-balances packets, so single-shot commands can be lost.
// Resend until the FCU confirms state change or ACK is seen.
int arm_with_retry(int arm, int max_ms) {
    g_last_ack[0] = 0;
    long long t0 = (long long)time(nullptr) * 1000L;
    while (g_run) {
        send_arm_command(arm);
        int waited = 0;
        while (waited < 500 && g_run) {
            pump_until(nullptr, 50);
            // Command confirmed by ACK (result=0); state flag can lag one
            // heartbeat behind, so keep pumping to let HBs catch up.
            if (g_last_ack[0] && strstr(g_last_ack, "result=0")) {
                long long hbWait = 0;
                while (g_run && g_armed != (arm == 1) && hbWait < 3000) {
                    pump_until(nullptr, 50);
                    hbWait += 50;
                }
                if (g_armed == (arm == 1)) return 0;
            }
            if (g_armed == (arm == 1)) return 0;
            waited += 50;
        }
        long long now = (long long)time(nullptr) * 1000L;
        if (now - t0 > max_ms) return -1;
    }
    return -1;
}

// --- test runners ----------------------------------------------------------
void z_sweep() {
    const int steps[] = {0, 250, 500, 750, 1000};

    send_set_mode(0);
    pump_until([]() { return g_mode == 0; }, 3000);
    printf("mode=MANUAL (%u), armed=%d\n", g_mode, g_armed ? 1 : 0);

    if (arm_with_retry(1, 10000) != 0) { printf("FAILED to arm\n"); return; }
    printf("ARMED. ACK:%s\n", g_last_ack[0] ? g_last_ack : "(none)");
    if (!g_armed) { printf("FAILED to arm\n"); return; }

    long mc_us = 50 * 1000;                    // stream spacing (MC_US env)
    if (getenv("MC_US")) mc_us = atol(getenv("MC_US"));
    const long dwell_us = 4L * 1000 * 100;  // ~4s per step at any rate
    for (int z : steps) {
        for (long sent = 0; g_run && sent * mc_us < dwell_us; sent++) {
            send_manual_control(0, 0, z, 0);
            usleep(mc_us);
        }
        // read the freshest SERVO_OUTPUT_RAW
        g_servo_valid = false;
        pump_until([]() { return g_servo_valid; }, 1000);
        printf("z=%4d  AUX[1-6]=%5d %5d %5d %5d %5d %5d\n",
               z, g_servo[8], g_servo[9], g_servo[10], g_servo[11],
               g_servo[12], g_servo[13]);
    }

    // long re-hold at neutral to check the vertical pair settles (matches the
    // long z=500 holds Python uses for its baselines)
    for (long sent = 0; g_run && sent * mc_us < 2 * dwell_us; sent++) {
        send_manual_control(0, 0, 500, 0);
        usleep(mc_us);
    }
    g_servo_valid = false;
    pump_until([]() { return g_servo_valid; }, 1000);
    printf("z=%4d  AUX[1-6]=%5d %5d %5d %5d %5d %5d   <- re-hold\n",
           500, g_servo[8], g_servo[9], g_servo[10], g_servo[11],
           g_servo[12], g_servo[13]);

    // neutral + disarm
    for (int i = 0; i < 10; i++) { send_manual_control(0, 0, 500, 0); usleep(50 * 1000); }
    if (arm_with_retry(0, 10000) == 0) { printf("DISARMED. ACK:%s\n", g_last_ack[0] ? g_last_ack : "(none)"); }
    else { printf("WARN: disarm failed, bang appropriate!\n"); }
}

void rc_ovr_sweep() {
    // Mirrors the pymavlink per-axis RC_CHANNELS_OVERRIDE verification.
    const uint16_t NEUTRAL[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

    send_set_mode(0);
    pump_until([]() { return g_mode == 0; }, 3000);
    printf("mode=MANUAL (%u), armed=%d\n", g_mode, g_armed ? 1 : 0);
    if (arm_with_retry(1, 10000) != 0) { printf("FAILED to arm\n"); return; }
    printf("ARMED. ACK:%s\n", g_last_ack[0] ? g_last_ack : "(none)");
    if (!g_armed) { printf("FAILED to arm\n"); return; }

    auto hold = [&](const uint16_t ch[8], long ms) {
        long mc_us2 = 50 * 1000;
        if (getenv("MC_US")) mc_us2 = atol(getenv("MC_US"));
        const long steps = ms * 1000 / (mc_us2 > 0 ? mc_us2 : 1);
        for (long i = 0; g_run && i < steps; i++) {
            send_rc_override(ch);
            usleep(mc_us2);
        }
        g_servo_valid = false;
        pump_until([]() { return g_servo_valid; }, 1000);
    };

    hold(NEUTRAL, 6000);
    const uint16_t base[6] = {g_servo[8], g_servo[9], g_servo[10],
                              g_servo[11], g_servo[12], g_servo[13]};
    printf("neutral base      AUX1-6=%5u %5u %5u %5u %5u %5u\n",
           base[0], base[1], base[2], base[3], base[4], base[5]);

    struct test { const char *name; uint16_t ch[8]; };
    uint16_t ch_tmp[8];
    test tests[] = {
        {"ch2 pitch=1750 surge_forward", {1500, 1750, 1500, 1500, 1500, 1500, 1500, 1500}},
        {"ch2 pitch=1250 surge_backw",   {1500, 1250, 1500, 1500, 1500, 1500, 1500, 1500}},
        {"ch1 roll=1750  sway_right",    {1750, 1500, 1500, 1500, 1500, 1500, 1500, 1500}},
        {"ch1 roll=1250  sway_left",     {1250, 1500, 1500, 1500, 1500, 1500, 1500, 1500}},
        {"ch4 yaw=1750   yaw_right",     {1500, 1500, 1500, 1750, 1500, 1500, 1500, 1500}},
        {"ch4 yaw=1250   yaw_left",      {1500, 1500, 1500, 1250, 1500, 1500, 1500, 1500}},
        {"ch3 thrust=1750 heave_up",     {1500, 1500, 1750, 1500, 1500, 1500, 1500, 1500}},
        {"ch3 thrust=1250 heave_down",   {1500, 1500, 1250, 1500, 1500, 1500, 1500, 1500}},
    };
    for (auto &t : tests) {
        for (int i = 0; i < 8; i++) ch_tmp[i] = t.ch[i];
        hold(ch_tmp, 4000);
        printf("%-27s AUX[1-6]=%5u %5u %5u %5u %5u %5u  d=(%4d %4d %4d %4d %4d %4d)\n",
               t.name,
               g_servo[8], g_servo[9], g_servo[10], g_servo[11], g_servo[12], g_servo[13],
               g_servo[8] - base[0], g_servo[9] - base[1], g_servo[10] - base[2],
               g_servo[11] - base[3], g_servo[12] - base[4], g_servo[13] - base[5]);
    }

    hold(NEUTRAL, 6000);
    printf("neutral regain    AUX1-6=%5u %5u %5u %5u %5u %5u\n",
           g_servo[8], g_servo[9], g_servo[10], g_servo[11], g_servo[12], g_servo[13]);
    if (arm_with_retry(0, 10000) == 0) printf("DISARMED. ACK:%s\n", g_last_ack[0] ? g_last_ack : "(none)");
}

void interactive() {
    printf("C++ interactive MANUAL_CONTROL tester (QGC-mirror packing).\n");
    printf("Keys: w/s surge, a/d sway, r/f heave(z=1000/0), q/e yaw, z=hover(500), k=neutral, 1=arm 0=disarm, m=MANUAL, q=quit\n");
    send_arm_command(1);
    pump_until([]() { return g_armed; }, 8000);
    printf("armed=%d\n", g_armed ? 1 : 0);

    int16_t x = 0, y = 0, z = 500, r = 0;
    while (g_run) {
        send_manual_control(x, y, z, r);
        pump_until(nullptr, 30);
        if (g_servo_valid) {
            printf("\r%s armed=%d  cmd x=%5d y=%5d z=%5d r=%5d  AUX[1-6]=%5d %5d %5d %5d %5d %5d  ",
                   g_armed ? "*" : " ", g_armed ? 1 : 0, x, y, z, r,
                   g_servo[8], g_servo[9], g_servo[10], g_servo[11], g_servo[12], g_servo[13]);
            fflush(stdout);
        }
    }
    send_arm_command(0);
    printf("\n");
}

}  // namespace

int main(int argc, char **argv) {
    bool doSweep = false, doInteractive = false, doRcovr = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sweep")) doSweep = true;
        else if (!strcmp(argv[i], "--interactive")) doInteractive = true;
        else if (!strcmp(argv[i], "--rcovr")) doRcovr = true;
        // --modern/--legacy/--frame1/--frame2 are accepted but ignored: the
        // sender always emits hand-built MAVLink1 legacy frames (byte-equal to
        // pymavlink v1), the only form verified to decode on ArduSub v4.7.
    }
    if (!doSweep && !doInteractive && !doRcovr) doSweep = true;

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_fd < 0) { perror("socket"); return 1; }

    // Bind 127.0.0.1:14555 with SO_REUSEADDR ONLY (exactly what pymavlink's
    // 'udpin:' does). Mixing in SO_REUSEPORT would put us in the kernel's
    // load-balance group vs. the router and we'd stop receiving the FCU
    // stream. With plain SO_REUSEADDR the last binder owns the quadruple and
    // receives everything the router is not consuming.
    int one = 1;
    setsockopt(g_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    g_router = {};
    g_router.sin_family = AF_INET;
    g_router.sin_port = htons(14555);
    inet_pton(AF_INET, "127.0.0.1", &g_router.sin_addr);

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons(14555);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(g_fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
        perror("bind");
        return 1;
    }

    // heartbeat thread (HB=0 disables it, for sender-difference experiments)
    std::thread hb([&]() {
        if (getenv("HB") && !strcmp(getenv("HB"), "0")) return;
        while (g_run) { send_heartbeat(); usleep(1 * 1000 * 1000); }
    });

    // wait for FCU heartbeat
    pump_until([]() { return g_mode != UINT32_MAX; }, 5000);
    if (g_mode == UINT32_MAX) { printf("NO FCU HEARTBEAT\n"); return 1; }

    if (doSweep) z_sweep();
    if (doRcovr) {
        printf("[peer] g_peer_learned=%d port=%u (router=%u)\n",
               g_peer_learned ? 1 : 0, ntohs(g_peer.sin_port), ntohs(g_router.sin_port));
        rc_ovr_sweep();
    }
    if (doInteractive) interactive();

    g_run = false;
    hb.join();
    close(g_fd);
    return 0;
}