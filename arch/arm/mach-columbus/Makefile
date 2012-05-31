#
# Makefile for the Columbus Compute subsystem
#

obj-y				:= columbus.o
obj-$(CONFIG_COLUMBUS_MODEL)    += board-rtsm.o
obj-$(CONFIG_COLUMBUS_TC2)	+= board-tc2.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o columbus-smc.o
obj-$(CONFIG_HOTPLUG)		+= hotplug.o hotplug-asm.o
obj-$(CONFIG_CPU_FREQ_COLUMBUS)	+= cpufreq.o
