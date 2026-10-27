/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Small target libc replacements. Built outside LTO, with loop-pattern recognition
 * disabled so GCC cannot turn these loops back into calls to themselves. Word
 * accesses are aligned and alias-safe; differing alignments use the byte path.
 * Digital-M's main-loop copies use that compact byte path for every alignment. */
#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
typedef uint32_t mem_word_t __attribute__((__may_alias__));
#else
typedef uint32_t mem_word_t;
#endif

void *memcpy(void *restrict dst, const void *restrict src, size_t n)
{
    unsigned char *d = dst;
    const unsigned char *s = src;
    unsigned char *end = d + n;
#if !defined(BOARD) || BOARD != 3
    if ((((uintptr_t)d | (uintptr_t)s) & 3u) == 0u) {
        unsigned char *words = d + (n & ~(size_t)15);
        while (d != words) {
            ((mem_word_t *)d)[0] = ((const mem_word_t *)s)[0];
            ((mem_word_t *)d)[1] = ((const mem_word_t *)s)[1];
            ((mem_word_t *)d)[2] = ((const mem_word_t *)s)[2];
            ((mem_word_t *)d)[3] = ((const mem_word_t *)s)[3];
            d += 16;
            s += 16;
        }
        words = d + (n & 12u);
        while (d != words) {
            *(mem_word_t *)d = *(const mem_word_t *)s;
            d += 4;
            s += 4;
        }
    }
#endif
    while (d != end) {
        *d++ = *s++;
    }
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    unsigned char *d = dst;
    unsigned char *end = d + n;
    /* Receiver pages clear 48-byte structs and a possibly unaligned 24-byte
     * answer. Align the prefix, then fill four words per iteration. */
    while (((uintptr_t)d & 3u) != 0u && d != end) {
        *d++ = (unsigned char)c;
        n--;
    }
    if (n >= 4u) {
        mem_word_t w = (unsigned char)c * 0x01010101u;
        unsigned char *words = d + (n & ~(size_t)15);
        while (d != words) {
            ((mem_word_t *)d)[0] = w;
            ((mem_word_t *)d)[1] = w;
            ((mem_word_t *)d)[2] = w;
            ((mem_word_t *)d)[3] = w;
            d += 16;
        }
        words = d + (n & 12u);
        while (d != words) {
            *(mem_word_t *)d = w;
            d += 4;
        }
    }
    while (d != end) {
        *d++ = (unsigned char)c;
    }
    return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
    unsigned char *d = dst;
    const unsigned char *s = src;
    /* Integer addresses avoid ordering pointers into unrelated objects. The
     * forward path is valid for non-overlap or a destination below the source. */
    if ((uintptr_t)d <= (uintptr_t)s || (uintptr_t)d - (uintptr_t)s >= n) {
        while (n-- != 0u) {
            *d++ = *s++;
        }
    } else {
        d += n;
        s += n;
        while (n-- != 0u) {
            *--d = *--s;
        }
    }
    return dst;
}
