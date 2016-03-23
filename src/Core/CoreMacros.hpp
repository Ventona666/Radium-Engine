/*  ========================================================================= *
 *                              CoreMacros.hpp                                *
 *             core helper functions and macros for C and C++                 *
 *                          Valentin Roussellet                               *
 *  ========================================================================= */

// This file contains a host of helper functions and macros that should behave
// similarly on all platforms. They may be generally useful or helping to
// abstract compiler / architecture / platform specificities to write portable
// code when possible.

// A good reference : http://sourceforge.net/p/predef/

#ifndef RADIUMENGINE_CORE_HPP
#define RADIUMENGINE_CORE_HPP

#include <assert.h>
#include <stdio.h>

// ----------------------------------------------------------------------------
// Compiler identification
// ----------------------------------------------------------------------------
#if defined(__clang__)
#define COMPILER_CLANG
#elif defined(__GNUC__)
#define COMPILER_GCC
#elif defined (_MSC_VER)
#define COMPILER_MSVC
#else
#error unsupported compiler
#endif

// ----------------------------------------------------------------------------
// OS and architecture identification
// ----------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64)
#define OS_WINDOWS
#if defined(_M_X64)
#define ARCH_X64
#elif defined(_M_IX86)
#define ARCH_X86
#else
#error unsupported arch
#endif
#elif defined(__APPLE__) || defined(__MACH__)
#define OS_MACOS
#if defined(__i386__)
#define ARCH_X86
#elif defined(__x86_64__) || defined (__x86_64)
#define ARCH_X64
#else
#error unsupported arch
#endif
#elif defined(__linux__) || defined (__CYGWIN__)
#define OS_LINUX
#if defined(__i386__)
#define ARCH_X86
#elif defined(__x86_64__) || defined (__x86_64)
#define ARCH_X64
#else
#error unsupported arch
#endif
#else
#error unsupported OS
#endif

// Todo : endianness, pointer sixe

// ----------------------------------------------------------------------------
// Build configuration
// ----------------------------------------------------------------------------

// This will cause assert to be disabled except if DEBUG is defined
// Make sure all "debug" macros are defined
#if defined (DEBUG) || defined(_DEBUG) || defined (CORE_DEBUG)
#undef CORE_DEBUG
#define CORE_DEBUG

#undef _DEBUG
#define _DEBUG

#undef DEBUG
#define DEBUG

#undef NDEBUG
#undef RELEASE

#define ON_DEBUG(CODE) CODE
#else // not in debug

#define RELEASE

#undef CORE_DEBUG
#undef DEBUG
#undef _DEBUG

#if !defined (NDEBUG)
#define NDEBUG
#endif

#define ON_DEBUG(CODE) /* Nothing */
#endif

// ----------------------------------------------------------------------------
// Preprocessor magic
// ----------------------------------------------------------------------------

// Wrapper for multiline macros
// In debug we use the standard do..while(0) which is 'nice' to read and debug
// in a debugger such as gdb.
// In release we use the if(1) else;  which compilers can optimize better
#ifdef CORE_DEBUG
#define MACRO_START do {
#define MACRO_END   } while (0)
#else
#define MACRO_START if(1) {
#define MACRO_END   } else {}
#endif

// Macro to avoid the "unused variable" warning with no side-effects.
#define CORE_UNUSED(X) \
    MACRO_START        \
    (void) sizeof((X));\
    MACRO_END

// Token concatenation
// Preprocessor concatenation can be tricky if arguments are macros unless
// "recursively" calling concatenation through chained macros which forces
// the preprocessor to run another pass.
#define CONCATENATE(XX,YY) CONCATENATE2(XX,YY)
#define CONCATENATE2(XX,YY) CONCATENATE3(XX,YY)
#define CONCATENATE3(XX,YY) XX##YY

// Stringification has a similar problem.
#ifdef __STRING
#define STRINGIFY(X) __STRING(X)
#else
#define STRINGIFY(X) STRINGIFY2(X)
#define STRINGIFY2(X) #X
#endif

// ----------------------------------------------------------------------------
// Platform abstracted macros
// ----------------------------------------------------------------------------

// Breakpoints
// This macro will trigger a breakpoint where it is placed. With MSVC a dialog
// will ask you if you want to launch the debugger.
#if defined (COMPILER_GCC) || defined (COMPILER_CLANG)
#define BREAKPOINT(ARG) asm volatile ("int $3")
#elif defined (COMPILER_MSVC)
#define BREAKPOINT(ARG) __debugbreak()
#else
#error unsupported platform
#endif

// Platform-independent macros
// Note : there is support of deprecated and alignof in C++ 11

// Alignment
// ALIGN_OF : returns the alignment of a variable or field
// ALIGNED : declare an aligned variable

// Branch prediction hints (GCC & Clang only)
// UNLIKELY tells the compiler to expect the condition to be false
// LIKELY tells the compiler to expect the condition to be true

// Inlining commands
// ALWAYS_INLINE is the strongest. On GCC it will actually inline even
// when building without optimization (which is a bad idea most of the time).
// STRONG_INLINE is stronger than just "inline" where supported.
// NO_INLINE tells the compiler to never inline the function.

// DEPRECATED will issue a warning when using a variable or function.

// STDCALL, CDECL, FASTCALL : keyword for the corresponding calling convention.

#if defined (COMPILER_MSVC)

#define ALIGN_OF(X) __alignof(X)
#define ALIGNED(DECL,ALIGN) __declspec(align(ALIGN)) DECL

// Unfortunately visual studio does not have a branch prediction primitive.
#define UNLIKELY(IFEXPR) IFEXPR
#define LIKELY(IFEXPR)   IFEXPR

#define ALWAYS_INLINE __forceinline
#define STRONG_INLINE __forceinline
#define NO_INLINE     __declspec(noinline)

#define DEPRECATED __declspec(deprecated)

#define DLL_EXPORT __declspec(dllexport)
#define DLL_IMPORT __declspec(dllimport)

#define STDCALL __stdcall
#define CDECL __cdecl
#define FASTCALL __fastcall
#elif defined(COMPILER_GCC) || defined (COMPILER_CLANG)

#define ALIGN_OF(X) __alignof__(X)
#define ALIGNED(DECL,ALIGN) DECL __attribute__((aligned(ALIGN)))

#define UNLIKELY(IFEXPR) __builtin_expect(bool(IFEXPR),0)
#define LIKELY(IFEXPR)   __builtin_expect(bool(IFEXPR),1)

#define ALWAYS_INLINE  __attribute((always_inline))
#define STRONG_INLINE  inline
#define NO_INLINE     __attribute__((noinline))

#define DEPRECATED __attribute__((deprecated))

#define DLL_EXPORT
#define DLL_IMPORT
#define STDCALL  __attribute__((stdcall))
#define CDECL /* default */
#define FASTCALL __attribute__((fastcall))
#else
#error unsupported platform
#endif


// ----------------------------------------------------------------------------
// Useful typedefs
// ----------------------------------------------------------------------------

typedef unsigned char   uchar;
typedef unsigned short  ushort;
typedef unsigned int    uint;
typedef unsigned long   ulong;

// Use this to use double precision for all maths
// #define CORE_USE_DOUBLE
#ifndef CORE_USE_DOUBLE
typedef float Scalar;
#else
typedef double Scalar;
#endif

// ----------------------------------------------------------------------------
// Debug tools
// ----------------------------------------------------------------------------

// Static assert (works only with C++)
// (note : there is static assert in C++11)
namespace compile_time_utils
{
    template<bool b> struct error;
    template<> struct error<true> {};

    template<int x> struct size;
}
// This macro will output a compiler error if the argument evaluates to false
// at compile time.
#define STATIC_ASSERT(...) \
    typedef compile_time_utils::error <static_cast<bool>(__VA_ARGS__)>  \
    CONCATENATE(static_assert_line_, __LINE__)

// This macro will print the size of a type in a compiler error
// Note : there is a way to print it as a warning instead on StackOverflow
#define STATIC_SIZEOF(TYPE) compile_time_utils::size<sizeof(TYPE)> static_sizeof

// Custom assert, warn and error macros.
// Standard assert has two main drawbacks : on some OSes it aborts the program,
// and the debugger will break in assert.c or and not where the calling code
// uses assert(). CORE_ASSERT guarantees a breakpoint when in a debugger,
// and always prints a useful message.
// CORE_WARN_IF has the same effect but it will only print a message.
#ifdef CORE_DEBUG
#define CORE_ASSERT( EXP, DESC )                       \
    MACRO_START                                        \
    if (UNLIKELY(!(EXP))) {                            \
        fprintf(stderr,                                \
                "%s:%i: Assertion `%s` failed : %s\n", \
                __FILE__,__LINE__, #EXP, DESC);        \
        BREAKPOINT(0);                                 \
    } else {}                                          \
    MACRO_END

#define CORE_WARN_IF( EXP, DESC )                      \
    MACRO_START                                        \
    if (UNLIKELY((EXP))) {                             \
        fprintf(stderr,                                \
                "%s:%i: WARNING `%s` : %s\n",          \
                __FILE__,__LINE__, #EXP, DESC);        \
    } else{}                                           \
    MACRO_END
#else
#define CORE_ASSERT( EXP, DESC ) // nothing
#define CORE_WARN_IF( EXP, DESC ) // nothing
#endif

// Print an error and break, even in release.
#define CORE_ERROR( DESC )                  \
    MACRO_START                             \
    fprintf(stderr,                         \
            "%s:%i: ERROR : %s\n",          \
            __FILE__,__LINE__, DESC);       \
    BREAKPOINT(0);                          \
    MACRO_END

// Print an error and break if condition is not met, even in release
#define CORE_ERROR_IF( EXP, DESC )          \
    MACRO_START                             \
    if( UNLIKELY(!(EXP)) ) {                \
        CORE_ERROR( DESC );                 \
    }                                       \
    MACRO_END



// ----------------------------------------------------------------------------
// Explicit compiler warning disables.
// ----------------------------------------------------------------------------

// With the most sensitive warning settings, using this file can trigger lots
// of unwanted warnings, so we explicitly disable them. Add more at your
// own risk...

#if defined(COMPILER_GCC)
// Triggered by the typedef in static assert.
    #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
#if defined(COMPILER_MSVC)
    #pragma warning(disable: 4244) // Conversion from double to float loses data.
    #pragma warning(disable: 4251) // stl dllexports
    #pragma warning(disable: 4267) // conversion from size_t to uint
    #pragma warning(disable: 4577) // noexcept used with no exception handling mode
    #pragma warning(disable: 4838) // conversion from enum to uint.
    #pragma warning(disable: 4996) // sprintf unsafe
    #define NOMINMAX
    #include <windows.h>
#endif

#define eigen_assert(XXX) CORE_ASSERT(XXX, "Eigen Assert");

#endif // RADIUMENGINE_CORE_HPP
