#ifndef BALLOON_VISUAL_GUARD_H
#define BALLOON_VISUAL_GUARD_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "utils/cJSON.h"

#ifndef BALLOON_VISUAL_STATUS_FILE
#define BALLOON_VISUAL_STATUS_FILE "/tmp/balloon_visual_status.json"
#endif
#ifndef BALLOON_VISUAL_ACK_FILE
#define BALLOON_VISUAL_ACK_FILE "/tmp/balloon_mission_ack.json"
#endif
#define BALLOON_VISUAL_PREFLIGHT_MS 5000ULL
#define BALLOON_VISUAL_LOST_MS 3000ULL
#define BALLOON_VISUAL_FRAME_MAX_AGE_MS 1400ULL

typedef struct {
    char token[96], clockId[80], planId[96], color[24];
    uint64_t index, startedMs;
} T_BalloonVisualContext;

typedef struct {
    T_BalloonVisualContext context;
    uint64_t captureMs, finishedMs;
    uint32_t seq;
    bool targetFound;
    char source[24];
    float width, height, x1, y1, x2, y2, confidence;
} T_BalloonVisualReceipt;

/* Use the same installed three-color classifier groups as balloon_matches.
 * The original requested color remains exact in both mission receipts. */
static inline int BalloonVisual_ColorGroup(const char *color)
{
    if (!strcmp(color, "any")) return 0;
    if (!strcmp(color, "pink") || !strcmp(color, "red") || !strcmp(color, "orange")) return 1;
    if (!strcmp(color, "yellow") || !strcmp(color, "green")) return 2;
    if (!strcmp(color, "blue") || !strcmp(color, "purple")) return 3;
    return -1;
}

static inline bool BalloonVisual_String(const cJSON *root, const char *key, char *out, size_t size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(item) || !item->valuestring[0] || strlen(item->valuestring) >= size) return false;
    snprintf(out, size, "%s", item->valuestring);
    return true;
}

static inline bool BalloonVisual_Number(const cJSON *root, const char *key, double *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *out = item->valuedouble;
    return true;
}

static inline bool BalloonVisual_Uint(const cJSON *root, const char *key, uint64_t *out)
{
    double value;
    if (!BalloonVisual_Number(root, key, &value) || value < 0 || value > 9007199254740991.0 || floor(value) != value) return false;
    *out = (uint64_t)value;
    return true;
}

/* Reject ambiguous duplicate keys and trailing garbage, including corrupted
 * partial writes; the worker publishes these small objects atomically. */
static inline cJSON *BalloonVisual_Parse(const char *text)
{
    cJSON *root = cJSON_ParseWithOpts(text, NULL, 1);
    if (!cJSON_IsObject(root)) { cJSON_Delete(root); return NULL; }
    const cJSON *item, *other;
    for (item = root->child; item; item = item->next) {
        if (!item->string) { cJSON_Delete(root); return NULL; }
        for (other = item->next; other; other = other->next) {
            if (other->string && !strcmp(item->string, other->string)) { cJSON_Delete(root); return NULL; }
        }
    }
    return root;
}

static inline cJSON *BalloonVisual_ReadJson(const char *path)
{
    char text[4096];
    FILE *fp = fopen(path, "r");
    if (!fp) return NULL;
    size_t length = fread(text, 1, sizeof(text) - 1, fp);
    bool complete = !ferror(fp) && feof(fp);
    fclose(fp);
    if (!complete || !length) return NULL;
    text[length] = 0;
    return BalloonVisual_Parse(text);
}

static inline bool BalloonVisual_Context(const cJSON *root, T_BalloonVisualContext *out)
{
    memset(out, 0, sizeof(*out));
    return BalloonVisual_String(root, "token", out->token, sizeof(out->token)) &&
        BalloonVisual_String(root, "clock_id", out->clockId, sizeof(out->clockId)) &&
        BalloonVisual_String(root, "plan_id", out->planId, sizeof(out->planId)) &&
        BalloonVisual_String(root, "requested_color", out->color, sizeof(out->color)) &&
        BalloonVisual_ColorGroup(out->color) >= 0 &&
        BalloonVisual_Uint(root, "current_index", &out->index) && out->index < 8 &&
        BalloonVisual_Uint(root, "started_ts_ms", &out->startedMs) && out->startedMs > 0;
}

static inline bool BalloonVisual_SameContext(const T_BalloonVisualContext *a, const T_BalloonVisualContext *b)
{
    return !strcmp(a->token, b->token) && !strcmp(a->clockId, b->clockId) &&
        !strcmp(a->planId, b->planId) && !strcmp(a->color, b->color) &&
        a->index == b->index && a->startedMs == b->startedMs;
}

static inline bool BalloonVisual_ParseAck(const cJSON *root, const char *token, const char *clock,
                                          uint64_t now, T_BalloonVisualContext *out)
{
    uint64_t stamp;
    char status[24];
    return root && BalloonVisual_Context(root, out) && !strcmp(out->token, token) &&
        !strcmp(out->clockId, clock) && BalloonVisual_String(root, "status", status, sizeof(status)) &&
        !strcmp(status, "loaded") && BalloonVisual_Uint(root, "ts_ms", &stamp) &&
        out->startedMs <= stamp && stamp <= now && now - stamp <= 2000;
}

static inline bool BalloonVisual_ParseReceipt(const cJSON *root, const T_BalloonVisualContext *context,
                                              uint64_t now, T_BalloonVisualReceipt *out)
{
    uint64_t version, seq;
    char source[24], selected[24];
    double value;
    memset(out, 0, sizeof(*out));
    if (!root || !BalloonVisual_Uint(root, "version", &version) || version != 1 ||
        !BalloonVisual_Context(root, &out->context) || !BalloonVisual_SameContext(context, &out->context) ||
        !BalloonVisual_String(root, "source", source, sizeof(source)) ||
        (strcmp(source, "yolo") && strcmp(source, "pixel")) ||
        !BalloonVisual_Uint(root, "capture_ts_ms", &out->captureMs) ||
        !BalloonVisual_Uint(root, "ts_ms", &out->finishedMs) ||
        out->captureMs < context->startedMs || out->captureMs > out->finishedMs ||
        out->finishedMs > now || now - out->captureMs > BALLOON_VISUAL_FRAME_MAX_AGE_MS ||
        !BalloonVisual_Uint(root, "seq", &seq) || seq >= 0x80000000ULL) return false;
    out->seq = (uint32_t)seq;
    snprintf(out->source, sizeof(out->source), "%s", source);
    if (!BalloonVisual_Number(root, "width", &value) || value <= 1 || value > 32768) return false;
    out->width = (float)value;
    if (!BalloonVisual_Number(root, "height", &value) || value <= 1 || value > 32768) return false;
    out->height = (float)value;
    const cJSON *found = cJSON_GetObjectItemCaseSensitive(root, "target_found");
    if (!cJSON_IsBool(found)) return false;
    out->targetFound = cJSON_IsTrue(found);
    if (!out->targetFound) return true;
    if (!cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "matches_requested"))) return false;
    if (!BalloonVisual_String(root, "selected_color", selected, sizeof(selected))) return false;
    int requestedGroup = BalloonVisual_ColorGroup(context->color), selectedGroup = BalloonVisual_ColorGroup(selected);
    if (selectedGroup <= 0 || (requestedGroup != 0 && selectedGroup != requestedGroup)) return false;
#define BALLOON_VISUAL_FLOAT(name) do { \
    if (!BalloonVisual_Number(root, #name, &value)) return false; \
    out->name = (float)value; \
} while (0)
    BALLOON_VISUAL_FLOAT(x1); BALLOON_VISUAL_FLOAT(y1);
    BALLOON_VISUAL_FLOAT(x2); BALLOON_VISUAL_FLOAT(y2); BALLOON_VISUAL_FLOAT(confidence);
#undef BALLOON_VISUAL_FLOAT
    return out->x1 >= 0 && out->y1 >= 0 && out->x2 <= out->width && out->y2 <= out->height &&
        out->x2 > out->x1 && out->y2 > out->y1 && out->confidence > 0 && out->confidence <= 1;
}

/* Only current mission ACKs authorize an action. Re-read after the visual
 * file so an action transition during file reads never approves an old box. */
static inline bool BalloonVisual_ReadContext(const char *token, const char *clock, uint64_t now,
                                             T_BalloonVisualContext *context)
{
    cJSON *ack = BalloonVisual_ReadJson(BALLOON_VISUAL_ACK_FILE);
    bool ok = BalloonVisual_ParseAck(ack, token, clock, now, context);
    cJSON_Delete(ack);
    return ok;
}

static inline bool BalloonVisual_ReadReceipt(const char *token, const char *clock, uint64_t now,
                                             const T_BalloonVisualContext *context,
                                             T_BalloonVisualReceipt *receipt)
{
    cJSON *root = BalloonVisual_ReadJson(BALLOON_VISUAL_STATUS_FILE);
    bool ok = BalloonVisual_ParseReceipt(root, context, now, receipt);
    cJSON_Delete(root);
    T_BalloonVisualContext after;
    return ok && BalloonVisual_ReadContext(token, clock, now, &after) &&
        BalloonVisual_SameContext(context, &after);
}

static inline bool BalloonVisual_TimedOut(uint64_t nowMono, uint64_t lastTargetMono)
{
    return nowMono >= lastTargetMono && nowMono - lastTargetMono >= BALLOON_VISUAL_LOST_MS;
}
#endif
