/*
 * AArch64- ILP32 specific system calls implementation
 *
 * Copyright (C) 2013 Cavium Inc.
 * Author: Andrew Pinski <apinski@cavium.com>
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

#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/compat.h>

/*
 * Wrappers to pass the pt_regs argument.
 */
asmlinkage long sys_rt_sigreturn_wrapper(void);
#define sys_rt_sigreturn        sys_rt_sigreturn_wrapper

/* Using Compat syscalls where necessary */
#define sys_ioctl		compat_sys_ioctl
/* iovec */
#define sys_readv		compat_sys_readv
#define sys_writev		compat_sys_writev
#define sys_preadv		compat_sys_preadv64
#define sys_pwritev		compat_sys_pwritev64
#define sys_vmsplice		compat_sys_vmsplice
/* robust_list_head */
#define sys_set_robust_list	compat_sys_set_robust_list
#define sys_get_robust_list	compat_sys_get_robust_list

/* kexec_segment */
#define sys_kexec_load		compat_sys_kexec_load

/* Ptrace has some structures which are different between ILP32 and LP64 */
#define sys_ptrace		compat_sys_ptrace

/* struct msghdr */
#define sys_recvfrom		compat_sys_recvfrom
#define sys_recvmmsg		compat_sys_recvmmsg
#define sys_sendmmsg		compat_sys_sendmmsg
#define sys_sendmsg		compat_sys_sendmsg
#define sys_recvmsg		compat_sys_recvmsg

#define sys_setsockopt		compat_sys_setsockopt
#define sys_getsockopt		compat_sys_getsockopt

/* Array of pointers */
#define sys_execve		compat_sys_execve
#define sys_move_pages		compat_sys_move_pages

/* iovec */
#define sys_process_vm_readv	compat_sys_process_vm_readv
#define sys_process_vm_writev	compat_sys_process_vm_writev

/* Pointer in struct */
#define sys_mount               compat_sys_mount

/* NUMA */
/* unsigned long bitmaps */
#define sys_get_mempolicy       compat_sys_get_mempolicy
#define sys_set_mempolicy       compat_sys_set_mempolicy
#define sys_mbind               compat_sys_mbind
/* array of pointers */
/* unsigned long bitmaps */
#define sys_migrate_pages       compat_sys_migrate_pages

/* Scheduler */
/* unsigned long bitmaps */
#define sys_sched_setaffinity   compat_sys_sched_setaffinity
#define sys_sched_getaffinity   compat_sys_sched_getaffinity

/* iov usage */
#define sys_keyctl              compat_sys_keyctl

/* aio */
/* Pointer to Pointer  */
#define sys_io_setup		compat_sys_io_setup
/* Array of pointers */
#define sys_io_submit           compat_sys_io_submit

/* We need to make sure the pointer gets copied correctly. */
asmlinkage long ilp32_sys_mq_notify(mqd_t mqdes, const struct sigevent __user *u_notification)
{
	struct sigevent __user *p = NULL;
	if (u_notification) {
		struct sigevent n;
		p = compat_alloc_user_space(sizeof(*p));
		if (copy_from_user(&n, u_notification, sizeof(*p)))
			return -EFAULT;
		if (n.sigev_notify == SIGEV_THREAD)
			n.sigev_value.sival_ptr = compat_ptr((uintptr_t)n.sigev_value.sival_ptr);
		if (copy_to_user(p, &n, sizeof(*p)))
			return -EFAULT;
	}
	return sys_mq_notify(mqdes, p);
}

/* sigevent contains sigval_t which is now 64bit always
   but need special handling due to padding for SIGEV_THREAD.  */
#define sys_mq_notify		ilp32_sys_mq_notify


/* sigaltstack needs some special handling as the
   padding for stack_t might not be non-zero. */
long ilp32_sys_sigaltstack(const stack_t __user *uss_ptr,
			   stack_t __user *uoss_ptr)
{
	stack_t uss, uoss;
	int ret;
	mm_segment_t seg;

	if (uss_ptr) {
		if (!access_ok(VERIFY_READ, uss_ptr, sizeof(*uss_ptr)))
			return -EFAULT;
		if (__get_user(uss.ss_sp, &uss_ptr->ss_sp) |
			__get_user(uss.ss_flags, &uss_ptr->ss_flags) |
			__get_user(uss.ss_size, &uss_ptr->ss_size))
			return -EFAULT;
		/* Zero extend the sp address and the size. */
		uss.ss_sp = (void *)(uintptr_t)(unsigned int)(uintptr_t)uss.ss_sp;
		uss.ss_size = (size_t)(unsigned int)uss.ss_size;
	}
	seg = get_fs();
	set_fs(KERNEL_DS);
	/* Note we need to use uoss as we have changed the segment to the
	   kernel one so passing an user one around is wrong. */
	ret = sys_sigaltstack((stack_t __force __user *) (uss_ptr ? &uss : NULL),
			      (stack_t __force __user *) &uoss);
	set_fs(seg);
	if (ret >= 0 && uoss_ptr)  {
		if (!access_ok(VERIFY_WRITE, uoss_ptr, sizeof(stack_t)) ||
		    __put_user(uoss.ss_sp, &uoss_ptr->ss_sp) ||
		    __put_user(uoss.ss_flags, &uoss_ptr->ss_flags) ||
		    __put_user(uoss.ss_size, &uoss_ptr->ss_size))
			ret = -EFAULT;
	}
	return ret;
}

/* sigaltstack needs some special handling as the padding
   for stack_t might not be non-zero. */
#define sys_sigaltstack		ilp32_sys_sigaltstack


#include <asm/syscall.h>

#undef __SYSCALL
#define __SYSCALL(nr, sym)	[nr] = sym,

asmlinkage long sys_mmap(unsigned long addr, unsigned long len,
			 unsigned long prot, unsigned long flags,
			 unsigned long fd, off_t off);

/*
 * The sys_call_ilp32_table array must be 4K aligned to be accessible from
 * kernel/entry.S.
 */
void *sys_call_ilp32_table[__NR_syscalls] __aligned(4096) = {
	[0 ... __NR_syscalls - 1] = sys_ni_syscall,
#include <asm/unistd.h>
};
