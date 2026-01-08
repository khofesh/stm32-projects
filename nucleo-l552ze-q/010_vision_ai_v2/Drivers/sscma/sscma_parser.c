/**
 * @file sscma_parser.c
 * @brief JSMN-based JSON parser implementation for SSCMA responses
 */

#define JSMN_HEADER
#include "jsmn.h"
#include "sscma_parser.h"
#include "sscma_stm32l5.h"
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * Static Token Buffer
 * ============================================================================ */

static jsmntok_t tokens[SSCMA_MAX_JSON_TOKENS];

/* ============================================================================
 * Private Helper Functions
 * ============================================================================ */

/**
 * @brief Compare token string with a key
 */
static bool tok_eq(const char *json, const jsmntok_t *tok, const char *key)
{
    if (tok->type != JSMN_STRING) return false;

    size_t key_len = strlen(key);
    size_t tok_len = tok->end - tok->start;

    if (key_len != tok_len) return false;

    return strncmp(json + tok->start, key, tok_len) == 0;
}

/**
 * @brief Copy token value to buffer
 */
static void tok_copy(const char *json, const jsmntok_t *tok,
                     char *buf, size_t buf_size)
{
    size_t len = tok->end - tok->start;
    if (len >= buf_size) len = buf_size - 1;

    memcpy(buf, json + tok->start, len);
    buf[len] = '\0';
}

/**
 * @brief Parse token as integer
 */
static int tok_to_int(const char *json, const jsmntok_t *tok)
{
    char buf[16];
    size_t len = tok->end - tok->start;

    if (len >= sizeof(buf)) len = sizeof(buf) - 1;

    memcpy(buf, json + tok->start, len);
    buf[len] = '\0';

    return atoi(buf);
}

/**
 * @brief Find key in object, return index of value token
 * @param start_tok Index of object token
 * @return Index of value token, or -1 if not found
 */
static int find_key(const char *json, const jsmntok_t *tokens, int num_tokens,
                    int start_tok, const char *key)
{
    if (tokens[start_tok].type != JSMN_OBJECT) return -1;

    int obj_size = tokens[start_tok].size;
    int idx = start_tok + 1;

    for (int i = 0; i < obj_size && idx < num_tokens; i++) {
        /* Check if this key matches */
        if (tok_eq(json, &tokens[idx], key)) {
            return idx + 1;  /* Return index of value */
        }

        /* Skip key */
        idx++;
        if (idx >= num_tokens) break;

        /* Skip value (and all its children) */
        int skip = 1;
        for (int j = 0; j < skip && idx < num_tokens; j++, idx++) {
            if (tokens[idx].type == JSMN_OBJECT || tokens[idx].type == JSMN_ARRAY) {
                skip += tokens[idx].size;
            }
        }
    }

    return -1;
}

/**
 * @brief Skip a token and all its children
 * @return Number of tokens to skip
 */
static int skip_token(const jsmntok_t *tokens, int num_tokens, int idx)
{
    int count = 1;

    if (tokens[idx].type == JSMN_OBJECT || tokens[idx].type == JSMN_ARRAY) {
        int children = tokens[idx].size;
        idx++;

        for (int i = 0; i < children && idx < num_tokens; i++) {
            int skip = skip_token(tokens, num_tokens, idx);
            count += skip;
            idx += skip;

            /* For objects, each "child" is a key-value pair */
            if (tokens[idx - skip].type == JSMN_OBJECT) {
                /* Value is next token */
                if (idx < num_tokens) {
                    skip = skip_token(tokens, num_tokens, idx);
                    count += skip;
                    idx += skip;
                }
            }
        }
    }

    return count;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int sscma_parse_response(const char *json, size_t json_len,
                         int *type, char *name, size_t name_size, int *code)
{
    if (!json || json_len == 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    /* Root must be an object */
    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "type" */
    if (type) {
        int idx = find_key(json, tokens, num_tokens, 0, "type");
        if (idx > 0 && tokens[idx].type == JSMN_PRIMITIVE) {
            *type = tok_to_int(json, &tokens[idx]);
        }
    }

    /* Find "name" */
    if (name && name_size > 0) {
        int idx = find_key(json, tokens, num_tokens, 0, "name");
        if (idx > 0 && tokens[idx].type == JSMN_STRING) {
            tok_copy(json, &tokens[idx], name, name_size);
        }
    }

    /* Find "code" */
    if (code) {
        int idx = find_key(json, tokens, num_tokens, 0, "code");
        if (idx > 0 && tokens[idx].type == JSMN_PRIMITIVE) {
            *code = tok_to_int(json, &tokens[idx]);
        }
    }

    return 0;
}

int sscma_parse_get_string(const char *json, size_t json_len,
                           const char *key, char *value, size_t value_size)
{
    if (!json || json_len == 0 || !key || !value) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    int idx = find_key(json, tokens, num_tokens, 0, key);
    if (idx < 0) return -1;

    if (tokens[idx].type == JSMN_STRING) {
        tok_copy(json, &tokens[idx], value, value_size);
        return 0;
    }

    return -1;
}

int sscma_parse_get_data_string(const char *json, size_t json_len,
                                const char *key, char *value, size_t value_size)
{
    if (!json || json_len == 0 || !key || !value) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0) return -1;

    /* If data is a string directly (like for ID?) */
    if (tokens[data_idx].type == JSMN_STRING && strcmp(key, "data") == 0) {
        tok_copy(json, &tokens[data_idx], value, value_size);
        return 0;
    }

    /* data should be an object */
    if (tokens[data_idx].type != JSMN_OBJECT) return -1;

    /* Find key within data */
    int idx = find_key(json, tokens, num_tokens, data_idx, key);
    if (idx < 0) return -1;

    if (tokens[idx].type == JSMN_STRING) {
        tok_copy(json, &tokens[idx], value, value_size);
        return 0;
    }

    return -1;
}

int sscma_parse_get_perf(const char *json, size_t json_len,
                         uint16_t *preprocess, uint16_t *inference,
                         uint16_t *postprocess)
{
    if (!json || json_len == 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0 || tokens[data_idx].type != JSMN_OBJECT) return -1;

    /* Find "perf" array within data */
    int perf_idx = find_key(json, tokens, num_tokens, data_idx, "perf");
    if (perf_idx < 0 || tokens[perf_idx].type != JSMN_ARRAY) return -1;

    int arr_size = tokens[perf_idx].size;
    if (arr_size < 3) return -1;

    int idx = perf_idx + 1;

    if (preprocess && idx < num_tokens) {
        *preprocess = (uint16_t)tok_to_int(json, &tokens[idx]);
    }
    idx++;

    if (inference && idx < num_tokens) {
        *inference = (uint16_t)tok_to_int(json, &tokens[idx]);
    }
    idx++;

    if (postprocess && idx < num_tokens) {
        *postprocess = (uint16_t)tok_to_int(json, &tokens[idx]);
    }

    return 0;
}

int sscma_parse_get_boxes(const char *json, size_t json_len,
                          sscma_box_t *boxes, int max_boxes)
{
    if (!json || json_len == 0 || !boxes || max_boxes <= 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0 || tokens[data_idx].type != JSMN_OBJECT) return 0;

    /* Find "boxes" array within data */
    int boxes_idx = find_key(json, tokens, num_tokens, data_idx, "boxes");
    if (boxes_idx < 0 || tokens[boxes_idx].type != JSMN_ARRAY) return 0;

    int arr_size = tokens[boxes_idx].size;
    int count = 0;
    int idx = boxes_idx + 1;

    for (int i = 0; i < arr_size && count < max_boxes && idx < num_tokens; i++) {
        /* Each box is an array: [x, y, w, h, score, target] */
        if (tokens[idx].type != JSMN_ARRAY || tokens[idx].size < 6) {
            idx += skip_token(tokens, num_tokens, idx);
            continue;
        }

        int box_arr_size = tokens[idx].size;
        idx++;  /* Move to first element */

        if (box_arr_size >= 6 && idx + 5 < num_tokens) {
            boxes[count].x = (uint16_t)tok_to_int(json, &tokens[idx]);
            boxes[count].y = (uint16_t)tok_to_int(json, &tokens[idx + 1]);
            boxes[count].w = (uint16_t)tok_to_int(json, &tokens[idx + 2]);
            boxes[count].h = (uint16_t)tok_to_int(json, &tokens[idx + 3]);
            boxes[count].score = (uint8_t)tok_to_int(json, &tokens[idx + 4]);
            boxes[count].target = (uint8_t)tok_to_int(json, &tokens[idx + 5]);
            count++;
        }

        idx += box_arr_size;  /* Skip to next box */
    }

    return count;
}

int sscma_parse_get_classes(const char *json, size_t json_len,
                            sscma_class_t *classes, int max_classes)
{
    if (!json || json_len == 0 || !classes || max_classes <= 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0 || tokens[data_idx].type != JSMN_OBJECT) return 0;

    /* Find "classes" array within data */
    int classes_idx = find_key(json, tokens, num_tokens, data_idx, "classes");
    if (classes_idx < 0 || tokens[classes_idx].type != JSMN_ARRAY) return 0;

    int arr_size = tokens[classes_idx].size;
    int count = 0;
    int idx = classes_idx + 1;

    for (int i = 0; i < arr_size && count < max_classes && idx < num_tokens; i++) {
        /* Each class is an array: [score, target] */
        if (tokens[idx].type != JSMN_ARRAY || tokens[idx].size < 2) {
            idx += skip_token(tokens, num_tokens, idx);
            continue;
        }

        int cls_arr_size = tokens[idx].size;
        idx++;  /* Move to first element */

        if (cls_arr_size >= 2 && idx + 1 < num_tokens) {
            classes[count].score = (uint8_t)tok_to_int(json, &tokens[idx]);
            classes[count].target = (uint8_t)tok_to_int(json, &tokens[idx + 1]);
            count++;
        }

        idx += cls_arr_size;  /* Skip to next class */
    }

    return count;
}

int sscma_parse_get_points(const char *json, size_t json_len,
                           sscma_point_t *points, int max_points)
{
    if (!json || json_len == 0 || !points || max_points <= 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0 || tokens[data_idx].type != JSMN_OBJECT) return 0;

    /* Find "points" array within data */
    int points_idx = find_key(json, tokens, num_tokens, data_idx, "points");
    if (points_idx < 0 || tokens[points_idx].type != JSMN_ARRAY) return 0;

    int arr_size = tokens[points_idx].size;
    int count = 0;
    int idx = points_idx + 1;

    for (int i = 0; i < arr_size && count < max_points && idx < num_tokens; i++) {
        /* Each point is an array: [x, y, score, target] */
        if (tokens[idx].type != JSMN_ARRAY || tokens[idx].size < 4) {
            idx += skip_token(tokens, num_tokens, idx);
            continue;
        }

        int pt_arr_size = tokens[idx].size;
        idx++;  /* Move to first element */

        if (pt_arr_size >= 4 && idx + 3 < num_tokens) {
            points[count].x = (uint16_t)tok_to_int(json, &tokens[idx]);
            points[count].y = (uint16_t)tok_to_int(json, &tokens[idx + 1]);
            points[count].z = 0;
            points[count].score = (uint8_t)tok_to_int(json, &tokens[idx + 2]);
            points[count].target = (uint8_t)tok_to_int(json, &tokens[idx + 3]);
            count++;
        }

        idx += pt_arr_size;  /* Skip to next point */
    }

    return count;
}

int sscma_parse_get_keypoints(const char *json, size_t json_len,
                              sscma_keypoints_t *keypoints, int max_keypoints,
                              int max_points_per_keypoint)
{
    if (!json || json_len == 0 || !keypoints || max_keypoints <= 0) return -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return -1;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return -1;

    /* Find "data" object */
    int data_idx = find_key(json, tokens, num_tokens, 0, "data");
    if (data_idx < 0 || tokens[data_idx].type != JSMN_OBJECT) return 0;

    /* Find "keypoints" array within data */
    int kp_idx = find_key(json, tokens, num_tokens, data_idx, "keypoints");
    if (kp_idx < 0 || tokens[kp_idx].type != JSMN_ARRAY) return 0;

    int arr_size = tokens[kp_idx].size;
    int count = 0;
    int idx = kp_idx + 1;

    for (int i = 0; i < arr_size && count < max_keypoints && idx < num_tokens; i++) {
        /* Each keypoint is: [[box], [[pt1], [pt2], ...]] */
        if (tokens[idx].type != JSMN_ARRAY || tokens[idx].size < 2) {
            idx += skip_token(tokens, num_tokens, idx);
            continue;
        }

        int outer_start = idx;
        idx++;  /* Move inside outer array */

        /* Parse box array */
        if (idx < num_tokens && tokens[idx].type == JSMN_ARRAY &&
            tokens[idx].size >= 6) {
            int box_idx = idx + 1;
            keypoints[count].box.x = (uint16_t)tok_to_int(json, &tokens[box_idx]);
            keypoints[count].box.y = (uint16_t)tok_to_int(json, &tokens[box_idx + 1]);
            keypoints[count].box.w = (uint16_t)tok_to_int(json, &tokens[box_idx + 2]);
            keypoints[count].box.h = (uint16_t)tok_to_int(json, &tokens[box_idx + 3]);
            keypoints[count].box.score = (uint8_t)tok_to_int(json, &tokens[box_idx + 4]);
            keypoints[count].box.target = (uint8_t)tok_to_int(json, &tokens[box_idx + 5]);

            idx += 1 + tokens[idx].size;  /* Skip box array */
        }

        /* Parse points array */
        if (idx < num_tokens && tokens[idx].type == JSMN_ARRAY) {
            int pts_arr_size = tokens[idx].size;
            int pt_count = 0;
            idx++;  /* Move inside points array */

            for (int j = 0; j < pts_arr_size && pt_count < max_points_per_keypoint &&
                 idx < num_tokens; j++) {
                if (tokens[idx].type == JSMN_ARRAY && tokens[idx].size >= 4) {
                    int pt_idx = idx + 1;
                    keypoints[count].points[pt_count].x =
                        (uint16_t)tok_to_int(json, &tokens[pt_idx]);
                    keypoints[count].points[pt_count].y =
                        (uint16_t)tok_to_int(json, &tokens[pt_idx + 1]);
                    keypoints[count].points[pt_count].z = 0;
                    keypoints[count].points[pt_count].score =
                        (uint8_t)tok_to_int(json, &tokens[pt_idx + 2]);
                    keypoints[count].points[pt_count].target =
                        (uint8_t)tok_to_int(json, &tokens[pt_idx + 3]);
                    pt_count++;
                }
                idx += 1 + tokens[idx].size;
            }

            keypoints[count].num_points = pt_count;
        } else {
            keypoints[count].num_points = 0;
        }

        /* Skip to next keypoint entry */
        idx = outer_start + skip_token(tokens, num_tokens, outer_start);
        count++;
    }

    return count;
}

bool sscma_parse_has_key(const char *json, size_t json_len, const char *key)
{
    if (!json || json_len == 0 || !key) return false;

    jsmn_parser parser;
    jsmn_init(&parser);

    int num_tokens = jsmn_parse(&parser, json, json_len, tokens, SSCMA_MAX_JSON_TOKENS);
    if (num_tokens < 0) return false;

    if (num_tokens == 0 || tokens[0].type != JSMN_OBJECT) return false;

    return find_key(json, tokens, num_tokens, 0, key) >= 0;
}
