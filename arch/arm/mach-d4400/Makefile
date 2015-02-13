obj-y := epit.o cpu.o

obj-$(CONFIG_DEBUG_LL) += lluart.o
obj-$(CONFIG_SOC_D4400) += clk-d4400.o mach-d4400.o restart.o simreset.o
obj-$(CONFIG_COMMON_CLK) += clk-pll.o clk-gate.o
obj-$(CONFIG_HAVE_D4400_GPC) += gpc.o
obj-$(CONFIG_GCR_DEV) +=scm.o
obj-$(CONFIG_SERDES_D4400) += serdes-d4400.o
obj-$(CONFIG_SRC_D4400) += src.o

ccflags-y += -DD4400_GIT_LAST_COMMIT_HASH="\"`git rev-parse HEAD`\""
ccflags-y += -DD4400_GIT_LAST_COMMIT_DATE="\"`git log -1 --format=%cd`\""
