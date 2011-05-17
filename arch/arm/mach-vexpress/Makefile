#
# Makefile for the linux kernel.
#

ifneq ($(CONFIG_ARCH_TUSCAN),y)
obj-y					:= v2m.o pm.o
endif
obj-$(CONFIG_ARCH_VEXPRESS_CA9X4)	+= ct-ca9x4.o
obj-$(CONFIG_ARCH_VEXPRESS_CA15X4)	+= ct-ca15x4.o
obj-$(CONFIG_ARCH_VEXPRESS_LT_ELBA)	+= lt-elba.o
obj-$(CONFIG_ARCH_TUSCAN)		+= board-tuscan.o
ifneq ($(CONFIG_ARCH_TUSCAN),y)
obj-$(CONFIG_SMP)			+= platsmp.o
endif
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_PCI)			+= pcie.o
obj-$(CONFIG_PCI_MSI)			+= msi.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o
