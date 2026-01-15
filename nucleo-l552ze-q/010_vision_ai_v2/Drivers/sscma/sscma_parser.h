/**
 * @file sscma_parser.h
 * @brief JSMN-based JSON parser for SSCMA responses
 *
 * Uses JSMN (https://github.com/zserge/jsmn) for tokenization
 * with static memory allocation.
 */

#ifndef SSCMA_PARSER_H
#define SSCMA_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward declaration - full definition in sscma_stm32l5.h */
typedef struct sscma_box_s sscma_box_t;
typedef struct sscma_class_s sscma_class_t;
typedef struct sscma_point_s sscma_point_t;
typedef struct sscma_keypoints_s sscma_keypoints_t;

/* ============================================================================
 * Configuration
 * ============================================================================ */

#ifndef SSCMA_MAX_JSON_TOKENS
#define SSCMA_MAX_JSON_TOKENS   256
#endif

/* ============================================================================
 * Functions
 * ============================================================================ */

/**
 * @brief Parse basic response fields (type, name, code)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param type Pointer to store response type (can be NULL)
 * @param name Buffer to store response name (can be NULL)
 * @param name_size Size of name buffer
 * @param code Pointer to store response code (can be NULL)
 * @return 0 on success, -1 on error
 */
int sscma_parse_response(const char *json, size_t json_len,
                         int *type, char *name, size_t name_size, int *code);

/**
 * @brief Get string value from JSON
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param key Key to find
 * @param value Buffer to store value
 * @param value_size Size of value buffer
 * @return 0 on success, -1 if key not found
 */
int sscma_parse_get_string(const char *json, size_t json_len,
                           const char *key, char *value, size_t value_size);

/**
 * @brief Get nested string value (data.key)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param key Key within "data" object
 * @param value Buffer to store value
 * @param value_size Size of value buffer
 * @return 0 on success, -1 if key not found
 */
int sscma_parse_get_data_string(const char *json, size_t json_len,
                                const char *key, char *value, size_t value_size);

/**
 * @brief Get performance data from JSON (data.perf array)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param preprocess Pointer to store preprocess time
 * @param inference Pointer to store inference time
 * @param postprocess Pointer to store postprocess time
 * @return 0 on success, -1 on error
 */
int sscma_parse_get_perf(const char *json, size_t json_len,
                         uint16_t *preprocess, uint16_t *inference,
                         uint16_t *postprocess);

/**
 * @brief Get boxes array from JSON (data.boxes)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param boxes Array to store boxes
 * @param max_boxes Maximum number of boxes
 * @return Number of boxes parsed, -1 on error
 */
int sscma_parse_get_boxes(const char *json, size_t json_len,
                          sscma_box_t *boxes, int max_boxes);

/**
 * @brief Get classes array from JSON (data.classes)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param classes Array to store classes
 * @param max_classes Maximum number of classes
 * @return Number of classes parsed, -1 on error
 */
int sscma_parse_get_classes(const char *json, size_t json_len,
                            sscma_class_t *classes, int max_classes);

/**
 * @brief Get points array from JSON (data.points)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param points Array to store points
 * @param max_points Maximum number of points
 * @return Number of points parsed, -1 on error
 */
int sscma_parse_get_points(const char *json, size_t json_len,
                           sscma_point_t *points, int max_points);

/**
 * @brief Get keypoints array from JSON (data.keypoints)
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param keypoints Array to store keypoints
 * @param max_keypoints Maximum number of keypoints
 * @param max_points_per_keypoint Maximum points per keypoint
 * @return Number of keypoints parsed, -1 on error
 */
int sscma_parse_get_keypoints(const char *json, size_t json_len,
                              sscma_keypoints_t *keypoints, int max_keypoints,
                              int max_points_per_keypoint);

/**
 * @brief Check if JSON contains a key
 * @param json JSON string
 * @param json_len Length of JSON string
 * @param key Key to find
 * @return true if key exists
 */
bool sscma_parse_has_key(const char *json, size_t json_len, const char *key);

#ifdef __cplusplus
}
#endif

#endif /* SSCMA_PARSER_H */
