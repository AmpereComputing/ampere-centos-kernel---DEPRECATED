/*
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2015 Cavium Inc.
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

/* For ILP32 AARCH64, we want to use the non compat names. */
#if defined(__aarch64__) && defined(__ILP32__)
#define __SYSCALL_NONCOMPAT
#endif

#define __ARCH_WANT_RENAMEAT

#include <asm-generic/unistd.h>
