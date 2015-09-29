#ifndef __ASM_POSIX_TYPES_H
#define __ASM_POSIX_TYPES_H

typedef unsigned short __kernel_old_uid_t;
typedef unsigned short __kernel_old_gid_t;
#define __kernel_old_uid_t __kernel_old_uid_t

#if defined(__ILP32__)
/* The ILP32 kernel ABI reuses LP64 system calls where possible; to
   this end, the equivalent ILP32 type definitions are used (a 64bit
   'long'-type in LP64 corresponds to a 'long long' in LP64).  */

typedef long long           __kernel_long_t;
typedef unsigned long long  __kernel_ulong_t;
#define __kernel_long_t     __kernel_long_t
#endif /* defined(__ILP32__) */

#include <asm-generic/posix_types.h>

#endif /* __ASM_POSIX_TYPES_H */
