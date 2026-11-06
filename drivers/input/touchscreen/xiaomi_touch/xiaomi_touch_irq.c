// SPDX-License-Identifier: GPL-2.0-or-later
/* Rodin IRQ affinity and frequency QoS, reconstructed from the stock module. */
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include "xiaomi_touch_common.h"

struct touch_irq_policy {
	struct cpufreq_policy *policy;
	struct freq_qos_request request;
};

struct touch_irq_state {
	cpumask_t normal_mask;
	cpumask_t game_mask;
	bool initialized;
	bool game;
	bool support[2];
	u32 *target[2];
	unsigned int target_count[2];
	struct touch_irq_policy *policies;
	unsigned int policy_count;
};

static DEFINE_MUTEX(irq_state_lock);
static struct touch_irq_state irq_states[XIAOMI_TOUCH_MAX_PANEL];

static void touch_irq_read_mask(struct device_node *np, const char *name,
			       struct cpumask *mask)
{
	u32 value;
	unsigned int cpu;

	cpumask_copy(mask, cpu_possible_mask);
	if (of_property_read_u32(np, name, &value) || !value)
		return;
	cpumask_clear(mask);
	for_each_possible_cpu(cpu)
		if (cpu < 32 && (value & BIT(cpu)))
			cpumask_set_cpu(cpu, mask);
	if (cpumask_empty(mask))
		cpumask_copy(mask, cpu_possible_mask);
}

static void touch_irq_remove_locked(struct touch_irq_state *state)
{
	unsigned int i;

	for (i = 0; i < state->policy_count; i++) {
		freq_qos_remove_request(&state->policies[i].request);
		cpufreq_cpu_put(state->policies[i].policy);
	}
	kfree(state->policies);
	kfree(state->target[0]);
	kfree(state->target[1]);
	memset(state, 0, sizeof(*state));
}

void init_touch_irq(int touch_id, struct device_node *np)
{
	static const char * const support[] = {
		"normal_boost_support", "game_boost_support",
	};
	static const char * const targets[] = {
		"normal_irq_boost_target", "game_irq_boost_target",
	};
	struct touch_irq_state *state;
	struct cpufreq_policy *policy;
	unsigned int cpu, i;
	int count, ret;

	if (touch_id < 0 || touch_id >= XIAOMI_TOUCH_MAX_PANEL || !np)
		return;
	mutex_lock(&irq_state_lock);
	state = &irq_states[touch_id];
	touch_irq_remove_locked(state);
	touch_irq_read_mask(np, "normal_cpu_mask", &state->normal_mask);
	touch_irq_read_mask(np, "game_cpu_mask", &state->game_mask);
	for (i = 0; i < ARRAY_SIZE(targets); i++) {
		if (!of_property_read_bool(np, support[i]))
			continue;
		count = of_property_count_u32_elems(np, targets[i]);
		if (count <= 0 || count > nr_cpu_ids)
			continue;
		state->target[i] = kcalloc(count, sizeof(u32), GFP_KERNEL);
		if (!state->target[i])
			goto fail;
		ret = of_property_read_u32_array(np, targets[i], state->target[i], count);
		if (ret)
			goto fail;
		state->target_count[i] = count;
		state->support[i] = true;
	}
	if (state->support[0] || state->support[1]) {
		state->policies = kcalloc(nr_cpu_ids, sizeof(*state->policies), GFP_KERNEL);
		if (!state->policies)
			goto fail;
		for_each_possible_cpu(cpu) {
			policy = cpufreq_cpu_get(cpu);
			if (!policy)
				continue;
			if (cpu != cpumask_first(policy->related_cpus)) {
				cpufreq_cpu_put(policy);
				continue;
			}
			i = state->policy_count;
			ret = freq_qos_add_request(&policy->constraints,
					&state->policies[i].request, FREQ_QOS_MIN, 0);
			if (ret < 0) {
				cpufreq_cpu_put(policy);
				goto fail;
			}
			state->policies[i].policy = policy;
			state->policy_count++;
		}
	}
	state->initialized = true;
	mutex_unlock(&irq_state_lock);
	return;
fail:
	touch_irq_remove_locked(state);
	mutex_unlock(&irq_state_lock);
}
EXPORT_SYMBOL_GPL(init_touch_irq);

void remove_touch_irq_boost(int touch_id)
{
	if (touch_id < 0 || touch_id >= XIAOMI_TOUCH_MAX_PANEL)
		return;
	mutex_lock(&irq_state_lock);
	touch_irq_remove_locked(&irq_states[touch_id]);
	mutex_unlock(&irq_state_lock);
}
EXPORT_SYMBOL_GPL(remove_touch_irq_boost);

void touch_irq_boost_switch(int touch_id, bool game)
{
	if (touch_id < 0 || touch_id >= XIAOMI_TOUCH_MAX_PANEL)
		return;
	mutex_lock(&irq_state_lock);
	irq_states[touch_id].game = game;
	mutex_unlock(&irq_state_lock);
}
EXPORT_SYMBOL_GPL(touch_irq_boost_switch);

void touch_irq_cpumask(int touch_id)
{
	struct touch_irq_state *state;

	if (touch_id < 0 || touch_id >= XIAOMI_TOUCH_MAX_PANEL)
		return;
	mutex_lock(&irq_state_lock);
	state = &irq_states[touch_id];
	if (state->initialized)
		set_cpus_allowed_ptr(current, state->game ? &state->game_mask :
				    &state->normal_mask);
	mutex_unlock(&irq_state_lock);
}
EXPORT_SYMBOL_GPL(touch_irq_cpumask);

static void touch_irq_set_boost(int touch_id, bool enable)
{
	struct touch_irq_state *state;
	unsigned int i, mode;
	u32 target;

	if (touch_id < 0 || touch_id >= XIAOMI_TOUCH_MAX_PANEL)
		return;
	mutex_lock(&irq_state_lock);
	state = &irq_states[touch_id];
	mode = state->game;
	for (i = 0; i < state->policy_count; i++) {
		target = 0;
		if (enable && state->support[mode] && i < state->target_count[mode]) {
			target = min_t(u32, state->target[mode][i], S32_MAX);
		}
		freq_qos_update_request(&state->policies[i].request, target);
	}
	mutex_unlock(&irq_state_lock);
}

void xiaomi_touch_irq_boost(int touch_id)
{
	touch_irq_set_boost(touch_id, true);
}

void xiaomi_touch_irq_boost_release(int touch_id)
{
	touch_irq_set_boost(touch_id, false);
}
