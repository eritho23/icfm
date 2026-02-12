#ifndef ARENA_H
#define ARENA_H

#include <base.h>

#define ARENA_BASE_POS (sizeof(mem_arena))
#define ARENA_ALIGN (sizeof(void*))

typedef struct {
    u64 reserve_size;
    u64 pos;
} mem_arena;

mem_arena* arena_create(u64 reserve_size, u64 commit_size);
void arena_free(mem_arena* arena);

void* arena_push(mem_arena* arena, u64 size, b32 non_zero);
void arena_pop(mem_arena* arena, u64 size);
void arena_pop_to(mem_arena* arena, u64 pos);
void arena_clear(mem_arena* arena);

#endif
