#pragma once

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 极简内存封装：helix 解码器只用 helix_malloc/helix_free，这里直接映射到标准 malloc/free。
 * 原 arduino-libhelix 用 Allocator + 日志，这里不引入，保持轻量。 */
static inline void *helix_malloc(int size)
{
    return malloc((size_t)size);
}

static inline void helix_free(void *ptr)
{
    free(ptr);
}

#ifdef __cplusplus
}
#endif
