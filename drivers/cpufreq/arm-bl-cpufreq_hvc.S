#include <linux/linkage.h>

#include "arm-bl-cpufreq.h"

.arch_extension virt

ENTRY(__arm_bl_get_cluster)
	hvc	#ARM_BL_HVC_GET_MPIDR
	bx	lr
ENDPROC(__arm_bl_get_cluster)

ENTRY(__arm_bl_switch_cluster)
	hvc	#ARM_BL_HVC_SWITCH_CLUSTER
	bx	lr
ENDPROC(__arm_bl_switch_cluster)
