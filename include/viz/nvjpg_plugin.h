#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *auto_battlebot_nvjpg_create(void);
void auto_battlebot_nvjpg_destroy(void *encoder);
size_t auto_battlebot_nvjpg_encode(void *encoder, const unsigned char *i420_full_range, int width,
                                   int height, int quality,
                                   const unsigned char **jpeg_until_next_call);

typedef void *(*AutoBattlebotNvjpgCreate)(void);
typedef void (*AutoBattlebotNvjpgDestroy)(void *);
typedef size_t (*AutoBattlebotNvjpgEncode)(void *, const unsigned char *, int, int, int,
                                           const unsigned char **);

#ifdef __cplusplus
}
#endif
