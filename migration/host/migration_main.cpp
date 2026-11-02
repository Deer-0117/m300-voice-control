// Separate host entry; original device main/application sources are not edited.
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <vector>
#include <unistd.h>
#include "application.hpp"
#include "dji_platform.h"
#include "m300_runtime.h"

static volatile sig_atomic_t stop_requested = 0;
static std::atomic<int> worker_status(0); // 0 starting, 1 running, 2 stopped, -1 failed
static T_DjiFlightControllerRidInfo rid_info;
static void on_signal(int) { stop_requested = 1; }

static bool number(const char *name, double *value)
{
    const char *text = std::getenv(name);
    if (!text || !*text) return false;
    char *end = NULL;
    errno = 0;
    const double parsed = std::strtod(text, &end);
    if (errno || !end || *end || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static bool read_verified_rid()
{
    // No coordinates are invented, defaulted or converted based on contradictory
    // examples. A later device-acceptance step must establish native SDK units.
    const char *confirmed = std::getenv("M300_RID_NATIVE_CONFIRMED");
    double latitude, longitude, altitude;
    if (!confirmed || std::strcmp(confirmed, "1") ||
        !number("M300_RID_LATITUDE_NATIVE", &latitude) ||
        !number("M300_RID_LONGITUDE_NATIVE", &longitude) ||
        !number("M300_RID_ALTITUDE_NATIVE", &altitude) ||
        altitude < 0 || altitude > 65535 || std::floor(altitude) != altitude) {
        return false;
    }
    rid_info.latitude = latitude;
    rid_info.longitude = longitude;
    rid_info.altitude = static_cast<uint16_t>(altitude);
    return true;
}

static void *runtime_task(void *)
{
    const T_DjiReturnCode result = M300Runtime_StartFromEnvironment(&rid_info);
    if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::fprintf(stderr, "M300 runtime initialization failed: 0x%llX. Restart required.\n",
                     static_cast<unsigned long long>(result));
        worker_status.store(-1);
        return NULL;
    }
    worker_status.store(1);
    while (!stop_requested) usleep(100000);
    M300Runtime_Stop();
    worker_status.store(2);
    return NULL;
}

int main(int argc, char **argv)
{
    // This branch is before Application construction and every explicit SDK call.
    if (argc == 2 && !std::strcmp(argv[1], "--build-info")) {
        std::puts("{\"host\":\"m300_voice_psdk_host\",\"ground_only\":true,"
                  "\"sdk_initialized\":false,\"flight_enabled\":false}");
        return 0;
    }
    if (argc < 2 || std::strcmp(argv[1], "--run")) {
        std::fprintf(stderr, "Usage: %s --build-info\nReal startup requires a later accepted --run configuration.\n", argv[0]);
        return 2;
    }
    // No movement/control authority in this release, even if inherited env says 1.
    if (setenv("M300_ALLOW_FLIGHT", "0", 1) != 0) return 2;
    if (!std::getenv("M300_RUNTIME_ROOT") || !read_verified_rid()) {
        std::fprintf(stderr, "Runtime root or operator-verified native RID is missing. No SDK initialization performed.\n");
        return 2;
    }
    std::vector<char *> application_args;
    application_args.push_back(argv[0]);
    for (int i = 2; i < argc; ++i) application_args.push_back(argv[i]);
    application_args.push_back(NULL);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        Application application(static_cast<int>(application_args.size()) - 1, application_args.data());
        T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        if (!osal || !osal->TaskCreate || !osal->TaskDestroy) return 1;
        T_DjiTaskHandle handle = NULL;
        const T_DjiReturnCode result = osal->TaskCreate("m300_runtime", runtime_task, 65536, NULL, &handle);
        if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return 1;
        while (worker_status.load() == 0 || worker_status.load() == 1) usleep(100000);
        osal->TaskDestroy(handle);
        return worker_status.load() == 2 ? 0 : 1;
    } catch (const std::exception &) {
        std::fprintf(stderr, "PSDK host startup failed. Inspect the local SDK log; do not retry in the same process.\n");
        return 1;
    }
}
