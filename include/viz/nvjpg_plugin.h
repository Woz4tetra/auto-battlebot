#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *auto_battlebot_nvjpg_create(void);
void auto_battlebot_nvjpg_destroy(void *encoder);
/** Encodes one 4:2:0 frame in I420 layout: contiguous full-range (JFIF) Y, Cb and Cr planes,
 *  even width and height. Points `*jpeg` at the result, valid until the next call on this
 *  encoder, and returns its size. Returns 0 on failure. */
size_t auto_battlebot_nvjpg_encode(void *encoder, const unsigned char *i420, int width, int height,
                                   int quality, const unsigned char **jpeg);

typedef void *(*AutoBattlebotNvjpgCreate)(void);
typedef void (*AutoBattlebotNvjpgDestroy)(void *);
typedef size_t (*AutoBattlebotNvjpgEncode)(void *, const unsigned char *, int, int, int,
                                           const unsigned char **);

#ifdef __cplusplus
}
#endif
