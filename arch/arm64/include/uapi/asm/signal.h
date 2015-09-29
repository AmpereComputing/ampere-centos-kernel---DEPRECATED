/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SIGNAL_H
#define __ASM_SIGNAL_H

/* Required for AArch32 compatibility. */
#define SA_RESTORER	0x04000000

#define MINSIGSTKSZ 5120
#define SIGSTKSZ    16384

/* For ILP32, sigset should be the same size fields as LP64 so use
   unsigned long long. */
#ifdef __ILP32__
#define __SIGSET_INNER_TYPE __extension__ unsigned long long
#define _NSIG_BPW 64

# ifdef __AARCH64EB__
#  define __SIGNAL_INNER(type, field)		\
	__extension__ struct {			\
		int __pad_##field;		\
		type field;			\
	} __attribute__((aligned(8)))
# else
#  define __SIGNAL_INNER(type, field)		\
	__extension__ struct {			\
		type field;			\
		int __pad_##field;		\
	} __attribute__((aligned(8)))
# endif

# define __SIGACTION_HANDLER(field)		\
	__SIGNAL_INNER(__sighandler_t, field)


#define __SIGACTION_FLAGS(field)		\
	__extension__ unsigned long long field

#define __SIGACTION_RESTORER(field)		\
	__SIGNAL_INNER(__sigrestore_t, field)

#endif

#include <asm-generic/signal.h>

#endif
