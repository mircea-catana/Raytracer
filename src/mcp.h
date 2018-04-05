#pragma once

#include <cinttypes>

#if defined(_WIN32) || defined(_WIN64)
#define MCP_PLATFORM_WINDOWS
#elif defined(__linux__)
#define MCP_PLATFORM_LINUX
#elif defined(__APPLE__)
#define MCP_PLATFORM_APPLE
#endif

#if !defined(MCP_PLATFORM_APPLE)
#include <malloc.h>
#endif

#if !defined(MCP_PLATFORM_WINDOWS) && !defined(MCP_PLATFORM_APPLE)
#include <alloca.h>
#endif

#if defined(MCP_PLATFORM_WINDOWS)
#define alloca _alloca
#endif

#ifndef MCP_L1_CACHE_LINE_SIZE
#define MCP_L1_CACHE_LINE_SIZE 64
#endif

#ifndef MCP_POINTER_SIZE
#if defined(__amd64__) || defined(_M_X64)
#define MCP_POINTER_SIZE 8
#elif defined(__i386__) || defined(_M_IX86)
#define MCP_POINTER_SIZE 4
#endif
#endif
