#ifndef __SRC_PRIV_H__
#define __SRC_PRIV_H__

struct src_regs {
	u32 sbmr;
	u32 srsr;
	u32 sscr;
	u32 scrcr;
	u32 srbr;
	u32 sgpr;
};

struct src_priv {
	struct device *dev;
	struct src_regs *regs;
	spinlock_t lock;
};

#endif

