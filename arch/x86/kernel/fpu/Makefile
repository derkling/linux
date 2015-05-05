#
# Build rules for the FPU support code:
#

obj-y					+= init.o bugs.o core.o regset.o signal.o xstate.o

# Make the measured functions as simple as possible:
CFLAGS_measure.o += -fomit-frame-pointer
CFLAGS_REMOVE_measure.o = -pg

obj-$(CONFIG_X86_DEBUG_FPU_PERFORMANCE) += measure.o
