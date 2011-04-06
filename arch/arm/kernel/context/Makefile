asflags-y	:= -mfpu=vfpv3
ccflags-y 	:= -fpic -DSECURE_ONLY=1 \
		 -DNONSECURE_ONLY=0

plus_sec	:= $(call as-instr,.arch_extension sec,+sec)
AFLAGS_v7.o	:= -Wa,-march=armv7-a$(plus_sec)

ldflags-y		:= -r

obj-y		  	:= a9_c.o gic.o sr_api.o \
		   	   translation.o a8.o a9.o entry.o \
		  	   helpers.o v7.o desc.o
obj-y 	+= scu.o pl310.o context.o platform.o power.o reset.o
