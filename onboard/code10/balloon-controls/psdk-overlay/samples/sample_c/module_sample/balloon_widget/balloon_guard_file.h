#ifndef BALLOON_GUARD_FILE_H
#define BALLOON_GUARD_FILE_H

#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* Runtime receipts must be small regular files. A special file must not
 * block the control loop, even before we can inspect the open descriptor. */
static inline bool BalloonGuard_ReadRegular(const char *path, char *text, size_t capacity)
{
    if (capacity < 2) return false;
    int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
    if (fd < 0) return false;

    struct stat status;
    bool valid = fstat(fd, &status) == 0 && S_ISREG(status.st_mode) &&
        status.st_size > 0 && (uintmax_t)status.st_size < capacity;
    size_t length = 0;
    while (valid && length < capacity) {
        ssize_t count = read(fd, text + length, capacity - length);
        if (count == 0) break;
        if (count > 0) length += (size_t)count;
        else valid = false;
    }
    (void)close(fd);
    if (!valid || length >= capacity || length != (size_t)status.st_size ||
        memchr(text, '\0', length)) return false;
    text[length] = '\0';
    return true;
}

#endif
