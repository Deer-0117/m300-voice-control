#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include "application.hpp"

int main(int argc, char **argv)
{
    try {
        Application application(argc, argv);
        while (true) sleep(1);
    } catch (...) {
        // Hardware may be powered off when the display service starts.
        // Return a normal failure so systemd can retry without a core dump.
        std::fprintf(stderr, "PSDK startup unavailable; waiting for aircraft and link readiness.\n");
        return EXIT_FAILURE;
    }
}
