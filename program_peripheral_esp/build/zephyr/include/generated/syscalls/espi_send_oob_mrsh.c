/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/espi.h>

extern int z_vrfy_espi_send_oob(const struct device * dev, struct espi_oob_packet * pckt);
uintptr_t z_mrsh_espi_send_oob(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg2;	/* unused */
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; const struct device * val; } parm0;
	parm0.x = arg0;
	union { uintptr_t x; struct espi_oob_packet * val; } parm1;
	parm1.x = arg1;
	int ret = z_vrfy_espi_send_oob(parm0.val, parm1.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}
