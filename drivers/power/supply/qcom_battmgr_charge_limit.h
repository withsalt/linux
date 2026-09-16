/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_BATTMGR_CHARGE_LIMIT_H
#define _QCOM_BATTMGR_CHARGE_LIMIT_H

struct qcom_battmgr_charge_limit {
	unsigned int start;
	unsigned int end;
	bool enabled;
	bool cutoff;
};

/* Thresholds are inclusive: resume at start, stop at end. */
static inline bool qcom_battmgr_charge_limit_valid(unsigned int start,
						 unsigned int end)
{
	return start < end && end <= 100;
}

/* Invalid readings must never release an enabled charge limit. */
static inline void qcom_battmgr_charge_limit_sample(struct qcom_battmgr_charge_limit *limit,
						   unsigned int soc, bool valid)
{
	if (!limit->enabled)
		limit->cutoff = false;
	else if (!valid || soc > 100 || soc >= limit->end)
		limit->cutoff = true;
	else if (soc <= limit->start)
		limit->cutoff = false;
}

#endif
