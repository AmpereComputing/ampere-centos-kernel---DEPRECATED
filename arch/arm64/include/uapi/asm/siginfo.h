/*
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2014 Cavium Inc.
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
#ifndef __ASM_SIGINFO_H
#define __ASM_SIGINFO_H

#define __ARCH_SI_PREAMBLE_SIZE	(4 * sizeof(int))

#ifdef __ILP32__
# ifdef __AARCH64EB__
#  define __SIGINFO_INNER(type, field)		\
		int __pad#field;		\
		type field
# else
#  define __SIGINFO_INNER(type, field)		\
		type field;			\
		int __pad#field
# endif

# undef __SIGINFO_VOIDPOINTER
# define __SIGINFO_VOIDPOINTER(field)		\
		__SIGINFO_INNER(void __user*, field)
# undef __SIGINFO_BAND

# define __SIGINFO_BAND(field)			\
	__SIGINFO_INNER(long, field)
#endif

#include <asm-generic/siginfo.h>

#endif
