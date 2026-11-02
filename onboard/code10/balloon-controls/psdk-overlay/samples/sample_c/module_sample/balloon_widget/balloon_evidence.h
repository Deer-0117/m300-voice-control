#ifndef BALLOON_EVIDENCE_H
#define BALLOON_EVIDENCE_H
#include <stdio.h>
#include <string.h>

/* Retained through Stop, cleared by reboot; this is attribution, never a gate. */
#define BALLOON_EVIDENCE_TOKEN "/tmp/balloon_evidence.token"
static inline void BalloonEvidence_Token(char token[96])
{
    token[0] = '\0';
    FILE *fp = fopen(BALLOON_EVIDENCE_TOKEN, "r");
    if (!fp) return;
    if (fgets(token, 96, fp) == NULL) token[0] = '\0';
    fclose(fp);
    token[strcspn(token, "\r\n")] = '\0';
    if (strspn(token, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") != strlen(token)) token[0] = '\0';
}

static inline void BalloonEvidence_Escape(const char *src, char *dst, size_t size)
{
    size_t n = 0;
    for (; src && *src && n + 2 < size; ++src) {
        unsigned char c = (unsigned char) *src;
        if (c == '"' || c == '\\') dst[n++] = '\\';
        if (c >= 32) dst[n++] = (char)c;
    }
    dst[n] = '\0';
}
#endif
