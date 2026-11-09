#ifndef BALLOON_API_MOTION_H
#define BALLOON_API_MOTION_H

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include "balloon_visual_guard.h"
#include "dji_flight_controller.h"
#include "target_chase/target_chase.h"

#define BALLOON_API_MOTION_HELPER "/home/uav/offline_balloon_strike_test/api_motion_bridge.py"
#define BALLOON_API_MOTION_REQUEST_MS 1400ULL

typedef struct {
    int fd;
    pid_t pid;
    size_t received;
    char response[512];
} T_BalloonApiMotionBridge;

typedef struct {
    T_DjiFlightControllerJoystickCommand joystick;
    uint64_t durationMs;
    char providerId[129];
} T_BalloonApiMotionCommand;

static inline bool BalloonApiMotion_Parse(const char *text, uint32_t seq,
                                           T_BalloonApiMotionCommand *command)
{
    if (strchr(text, '\\')) return false;
    cJSON *root = BalloonVisual_Parse(text);
    uint64_t responseSeq;
    double x, y, z, yaw;
    memset(command, 0, sizeof(*command));
    bool ok = root && cJSON_GetArraySize(root) == 7 &&
        BalloonVisual_Uint(root, "seq", &responseSeq) && seq > 0 && responseSeq == seq &&
        BalloonVisual_Uint(root, "duration_ms", &command->durationMs) &&
        (command->durationMs == 50 || command->durationMs == 100 || command->durationMs == 150) &&
        BalloonVisual_Number(root, "x_mps", &x) && fabs(x) <= 0.30 &&
        BalloonVisual_Number(root, "y_mps", &y) && hypot(x, y) <= 0.30 &&
        BalloonVisual_Number(root, "z_mps", &z) && fabs(z) <= 0.12 &&
        BalloonVisual_Number(root, "yaw_rate_degps", &yaw) && fabs(yaw) <= 12 &&
        BalloonVisual_String(root, "provider_id", command->providerId, sizeof(command->providerId)) &&
        command->providerId[strspn(command->providerId,
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")] == '\0';
    if (ok) {
        command->joystick.x = (float)x;
        command->joystick.y = (float)y;
        command->joystick.z = (float)z;
        command->joystick.yaw = (float)yaw;
    }
    cJSON_Delete(root);
    return ok;
}

static inline bool BalloonApiMotion_Start(T_BalloonApiMotionBridge *bridge)
{
    extern char **environ;
    int sockets[2];
#ifdef SOCK_CLOEXEC
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets) != 0) return false;
#else
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) return false;
    if (fcntl(sockets[0], F_SETFD, FD_CLOEXEC) || fcntl(sockets[1], F_SETFD, FD_CLOEXEC)) {
        close(sockets[0]);
        close(sockets[1]);
        return false;
    }
#endif
#ifndef MSG_NOSIGNAL
    int enabled = 1;
    if (setsockopt(sockets[0], SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled))) {
        close(sockets[0]);
        close(sockets[1]);
        return false;
    }
#endif
    /* Keep both original descriptors distinct from stdin/stdout. */
    if (sockets[0] < 3 || sockets[1] < 3) {
        close(sockets[0]);
        close(sockets[1]);
        return false;
    }
    posix_spawn_file_actions_t actions;
    int code = posix_spawn_file_actions_init(&actions);
    if (!code) {
        code = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDIN_FILENO);
        if (!code) code = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDOUT_FILENO);
        char *argv[] = {"/usr/bin/python3", "-I", BALLOON_API_MOTION_HELPER, NULL};
        if (!code) code = posix_spawn(&bridge->pid, argv[0], &actions, NULL, argv, environ);
        (void)posix_spawn_file_actions_destroy(&actions);
    }
    close(sockets[1]);
    if (code) {
        close(sockets[0]);
        return false;
    }
    bridge->fd = sockets[0];
    return true;
}

static inline void BalloonApiMotion_Close(T_BalloonApiMotionBridge *bridge)
{
    if (bridge->fd >= 0) close(bridge->fd);
    if (bridge->pid > 0) {
        int status;
        pid_t child;
        do {
            child = waitpid(bridge->pid, &status, WNOHANG);
        } while (child < 0 && errno == EINTR);
        if (child == 0) {
            (void)kill(bridge->pid, SIGKILL);
            while (waitpid(bridge->pid, &status, 0) < 0 && errno == EINTR) {}
        }
    }
}

static inline bool BalloonApiMotion_Request(T_BalloonApiMotionBridge *bridge, uint32_t seq,
                                            const T_BalloonVisualReceipt *vision,
                                            const T_TargetChaseAircraftState *aircraft)
{
    char json[512];
    int length = snprintf(json, sizeof(json),
        "{\"seq\":%u,\"target\":{\"u\":%.9g,\"v\":%.9g,\"width\":%.9g,\"height\":%.9g},"
        "\"aircraft\":{\"altitude_agl_m\":%.9g,\"vel_n_mps\":%.9g,\"vel_e_mps\":%.9g,"
        "\"vel_u_mps\":%.9g,\"yaw_deg\":%.9g}}\n",
        (unsigned)seq, ((double)vision->x1 + vision->x2) / (2.0 * vision->width),
        ((double)vision->y1 + vision->y2) / (2.0 * vision->height),
        ((double)vision->x2 - vision->x1) / vision->width,
        ((double)vision->y2 - vision->y1) / vision->height,
        aircraft->altitude_agl_m, aircraft->velocity_neu_mps[0], aircraft->velocity_neu_mps[1],
        aircraft->velocity_neu_mps[2], aircraft->attitude_deg[2]);
    bridge->received = 0;
    int flags = MSG_DONTWAIT;
#ifdef MSG_NOSIGNAL
    flags |= MSG_NOSIGNAL;
#endif
    return length > 0 && (size_t)length < sizeof(json) &&
        send(bridge->fd, json, (size_t)length, flags) == length;
}

/* EOF or unsolicited extra data invalidates the live command lease. */
static inline bool BalloonApiMotion_Idle(const T_BalloonApiMotionBridge *bridge)
{
    char unused;
    return recv(bridge->fd, &unused, 1, MSG_PEEK | MSG_DONTWAIT) < 0 &&
        (errno == EAGAIN || errno == EWOULDBLOCK);
}

/* Exactly one newline-terminated reply is allowed per request. A partial
 * reply never grants movement, and socket reads never block flight control. */
static inline int BalloonApiMotion_Poll(T_BalloonApiMotionBridge *bridge, uint32_t seq,
                                        T_BalloonApiMotionCommand *command)
{
    size_t left = sizeof(bridge->response) - bridge->received;
    ssize_t count = recv(bridge->fd, bridge->response + bridge->received, left, MSG_DONTWAIT);
    if (count < 0) return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ? 0 : -1;
    if (count == 0) return -1;
    bridge->received += (size_t)count;
    char *newline = memchr(bridge->response, '\n', bridge->received);
    if (!newline) return bridge->received < sizeof(bridge->response) ? 0 : -1;
    if (newline != bridge->response + bridge->received - 1 ||
        memchr(bridge->response, '\0', bridge->received)) return -1;
    *newline = '\0';
    return BalloonApiMotion_Parse(bridge->response, seq, command) ? 1 : -1;
}

#endif
