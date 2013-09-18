#ifndef __SRC_H__
#define __SRC_H__

#define SW_RST_MIN	1
#define SW_RST_MAX	8

enum rst_type {
	SW_TPIU_RST,
	SW_RST,
	SW_COLD_RST,
};

int src_assert_reset(void *src_handle, enum rst_type rst_type, int idx);
void *src_get_handle(struct device_node *src_dev_node);
int of_get_named_src_reset(struct device_node *np,
		struct of_phandle_args *phandle, const char *propname,
		int index);
#endif
