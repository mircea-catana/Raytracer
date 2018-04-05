#pragma once

#include "mcp.h"

namespace mcp
{
namespace memory
{
    void* allocAligned(size_t size);

    template <typename T>
    T* allocAligned(uint32_t count) {
        return (T*)allocAligned(count * sizeof(T));
    }

    void* allocAligned(size_t size) {
#if defined(MCP_PLATFORM_WINDOWS)
        return _aligned_malloc(size, MCP_L1_CACHE_LINE_SIZE);

#elif defined(MCP_PLATFORM_APPLE)
        void *mem  = malloc(size + (MCP_L1_CACHE_LINE_SIZE - 1) + sizeof(void*));
        char *amem = ((char*)mem) + sizeof(void*);

#if MCP_POINTER_SIZE == 8
        amem += MCP_L1_CACHE_LINE_SIZE - (reinterpret_cast<uint64_t>(amem) &
                                          (MCP_L1_CACHE_LINE_SIZE - 1));
#else
        amem += MCP_L1_CACHE_LINE_SIZE - (reinterpret_cast<uint32_t>(amem) &
                                          (MCP_L1_CACHE_LINE_SIZE - 1));
#endif

#else
        return memalign(MCP_L1_CACHE_LINE_SIZE, size);
#endif
    }

    void freeAligned(void* ptr) {
        if (!ptr) return;
#if defined(MCP_PLATFORM_WINDOWS)
        _aligned_free(ptr);
#elif defined(MCP_PLATFORM_APPLE)
        free(((void**)ptr)[-1]);
#else
        free(ptr);
#endif
    }

}
}
