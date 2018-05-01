/*
 * This can be used throughout hardware code to indicate that the hardware
 * is unsupported in RHEL.
 */
#include <linux/kernel.h>
#include <linux/module.h>

void mark_hardware_unsupported(const char *msg)
{
	/* Print one single message */
	pr_crit("Warning: %s - this hardware has not undergone testing by Red Hat and might not be certified. Please consult https://hardware.redhat.com for certified hardware.\n", msg);
}
EXPORT_SYMBOL(mark_hardware_unsupported);

void mark_hardware_deprecated(const char *msg)
{
	pr_crit("Warning: %s - this hardware is not recommended for new deployments. It continues to be supported in this RHEL release, but it is likely to be removed in the next major release. Driver updates and fixes for this device will be limited to critical issues. Please contact Red Hat Support or your device's hardware vendor for additional information.\n", msg);
}
EXPORT_SYMBOL(mark_hardware_deprecated);

/* Mark parts of the kernel as 'Tech Preview', to make it clear to our
 * support organization and customers what we do not fully support yet.
 * NOTE: this will TAINT the kernel to signify the machine is running
 * code that is not fully supported.  Use with caution.
 */
void mark_tech_preview(const char *msg, struct module *mod)
{
	const char *str = NULL;

	if (msg)
		str = msg;
	else if (mod && mod->name)
		str = mod->name;

	pr_warn("TECH PREVIEW: %s may not be fully supported.\n"
		"Please review provided documentation for limitations.\n",
		(str ? str : "kernel"));
	add_taint(TAINT_TECH_PREVIEW, LOCKDEP_STILL_OK);
	if (mod)
		mod->taints |= (1U << TAINT_TECH_PREVIEW);
}
EXPORT_SYMBOL(mark_tech_preview);

/**
 * mark_driver_unsupported - drivers that we know we don't want to support
 * @name: the name of the driver
 *
 * In some cases Red Hat has chosen to build a driver for internal QE
 * use. Use this function to mark those drivers as unsupported for
 * customers.
 */
void mark_driver_unsupported(const char *name)
{
	pr_crit("Warning: %s - This driver has not undergone sufficient testing by Red Hat for this release and therefore cannot be used in production systems.\n",
	        name ? name : "kernel");
}
EXPORT_SYMBOL(mark_driver_unsupported);
