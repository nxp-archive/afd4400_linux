#
# Makefile for the SFP and CPRI.
#

obj-$(CONFIG_RF_CPRI) += cpri_driver.o
cpri_driver-objs := cpri.o \
		cpri_ioctl.o\
		cpri_an.o \
		cpri_eth.o \
		cpri_cw.o

obj-$(CONFIG_RF_SFP) += sfp.o
