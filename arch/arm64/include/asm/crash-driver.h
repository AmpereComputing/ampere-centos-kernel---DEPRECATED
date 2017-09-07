#ifndef _ARM64_CRASH_H
#define _ARM64_CRASH_H

#include <asm-generic/crash-driver.h>

#define DEV_CRASH_ARCH_DATA _IOR('c', 1, long)

static long
crash_arch_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	extern u64 kimage_voffset;

	switch (cmd)
	{
	case DEV_CRASH_ARCH_DATA:
		return put_user(kimage_voffset, (unsigned long __user *)arg);
	default:
		return -EINVAL;
	}
}

#endif /* _ARM64_CRASH_H */
