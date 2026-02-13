#include "base.h"
#include "arena.h"
#include <esp_heap_caps.h>

static inline u64 align_up(u64 v, u64 a) {
    return (v + (a - 1)) & ~(a - 1);
}

mem_arena* arena_create(u64 size) {
    size = align_up(size, ARENA_ALIGN);

    mem_arena* arena = (mem_arena*)heap_caps_malloc(size, MALLOC_CAP_8BIT);

    if (!arena) return NULL;

    arena->reserve_size = size;
    arena->pos = ARENA_BASE_POS;

    return arena;
}

void arena_free(mem_arena* arena) {
    heap_caps_free(arena);
}


void* arena_push(mem_arena* arena, u64 size, b32 non_zero) {
    u64 pos_aligned = align_up(arena->pos, ARENA_ALIGN);
    u64 new_pos = pos_aligned + size;

    if (new_pos > arena->reserve_size)
        return NULL;

    void* out = (u8*)arena + pos_aligned;
    arena->pos = new_pos;

    if (!non_zero)
        memset(out, 0, size);

    return out;
}

void arena_pop(mem_arena* arena, u64 size) {
    u64 used = arena->pos - ARENA_BASE_POS;
    if (size > used) size = used;
    arena->pos -= size;
}

void arena_pop_to(mem_arena* arena, u64 pos) {
    if (pos < ARENA_BASE_POS) pos = ARENA_BASE_POS;
    if (pos < arena->pos) arena->pos = pos;
}

void arena_clear(mem_arena* arena) {
    arena->pos = ARENA_BASE_POS;
}
