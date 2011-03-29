#
# Makefile for the linux kernel.
#

obj-y					:= v2m.o pm.o
obj-$(CONFIG_ARCH_VEXPRESS_CA9X4)	+= ct-ca9x4.o
obj-$(CONFIG_ARCH_VEXPRESS_CA15X4)	+= ct-ca15x4.o
obj-$(CONFIG_ARCH_VEXPRESS_LT_ELBA)	+= lt-elba.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
obj-$(CONFIG_PCI)			+= pcie.o
obj-$(CONFIG_PCI_MSI)			+= msi.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o
