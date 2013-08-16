/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 2012 Enhanced by motley <motley.slate@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <mach/cpufreq.h>
#include <linux/reboot.h>

/*
 * Controls
 * DEFAULT_THROTTLE_TEMP - default throttle temp at boot time
 * MAX_THROTTLE_TEMP - max able to be set by user
 * COOL_TEMP - temp in C where where we can slow down polling
 * COOL_TEMP_OFFSET_MS - number of ms to add to polling time when temps are cool
 * HOT_TEMP_OFFSET_MS - number of ms to subtract from polling time when temps are hot
 * DEFAULT_MIN_FREQ_INDEX - frequency table index for the lowest frequency to drop to during throttling
 * */
#define DEFAULT_THROTTLE_TEMP		70
#define MAX_THROTTLE_TEMP			80
#define COOL_TEMP					45
#define COOL_TEMP_OFFSET_MS			250
#define HOT_TEMP_OFFSET_MS			250
#define DEFAULT_MIN_FREQ_INDEX		7

static int enabled;
static struct msm_thermal_data msm_thermal_info;
static uint32_t limited_max_freq = MSM_CPUFREQ_NO_LIMIT;
static struct delayed_work check_temp_work;
static bool core_control_enabled;
static uint32_t cpus_offlined;
static DEFINE_MUTEX(core_control_mutex);

static unsigned int limit_idx;
static unsigned int min_freq_index;
static unsigned int limit_idx_high;
static bool thermal_debug = false;
static bool throttle_on = false;
static unsigned int throttle_temp = DEFAULT_THROTTLE_TEMP;

static struct cpufreq_frequency_table *table;

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	int i = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_debug("%s: error reading cpufreq table\n", KBUILD_MODNAME);
		ret = -EINVAL;
		goto fail;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	min_freq_index = DEFAULT_MIN_FREQ_INDEX;
	limit_idx_high = limit_idx = i - 1;
	BUG_ON(limit_idx_high <= 0 || limit_idx_high <= min_freq_index);
fail:
	return ret;
}

static int update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;

	ret = msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, max_freq);
	if (ret)
		return ret;

	limited_max_freq = max_freq;
	if (max_freq != MSM_CPUFREQ_NO_LIMIT) {
		if (thermal_debug)
			pr_info("msm_thermal: limiting cpu%d max frequency to %d\n",
					cpu, max_freq);
	} else {
		if (thermal_debug)
			pr_info("msm_thermal: max frequency reset for cpu%d\n", cpu);
	}
	ret = cpufreq_update_policy(cpu);

	return ret;
}

/*static void __cpuinit do_core_control(long temp)
{
	int i = 0;
	int ret = 0;

	if (!core_control_enabled)
		return;

	**
	 *  Offline cores starting from the max MPIDR to 1, when above limit,
	 *  The core control mask is non zero and allows the core to be turned
	 *  off.
	 *  The core was not previously offlined by this module
	 *  The core is the next in sequence.
	 *  If the core was online for some reason, even after it was offlined
	 *  by this module, offline it again.
	 *  Online the back on if the temp is below the hysteresis and was
	 *  offlined by this module and not already online.
	 *
	mutex_lock(&core_control_mutex);
	if (msm_thermal_info.core_control_mask &&
		temp >= msm_thermal_info.core_limit_temp_degC) {
		for (i = num_possible_cpus(); i > 0; i--) {
			if (!(msm_thermal_info.core_control_mask & BIT(i)))
				continue;
			if (cpus_offlined & BIT(i) && !cpu_online(i))
				continue;
			pr_info("%s: Set Offline: CPU%d Temp: %ld\n",
					KBUILD_MODNAME, i, temp);
			ret = cpu_down(i);
			if (ret)
				pr_err("%s: Error %d offline core %d\n",
					KBUILD_MODNAME, ret, i);
			cpus_offlined |= BIT(i);
			break;
		}
	} else if (msm_thermal_info.core_control_mask && cpus_offlined &&
		temp <= (msm_thermal_info.core_limit_temp_degC -
			msm_thermal_info.core_temp_hysteresis_degC)) {
		for (i = 0; i < num_possible_cpus(); i++) {
			if (!(cpus_offlined & BIT(i)))
				continue;
			cpus_offlined &= ~BIT(i);
			pr_info("%s: Allow Online CPU%d Temp: %ld\n",
					KBUILD_MODNAME, i, temp);
			* If this core is already online, then bring up the
			 * next offlined core.
			 *
			if (cpu_online(i))
				continue;
			ret = cpu_up(i);
			if (ret)
				pr_err("%s: Error %d online core %d\n",
						KBUILD_MODNAME, ret, i);
			break;
		}
	}
	mutex_unlock(&core_control_mutex);
}*/

static void __cpuinit check_temp(struct work_struct *work)
{
	static int limit_init;
	struct tsens_device tsens_dev;
	long temp = 0;
	uint32_t max_freq = limited_max_freq;
	int cpu = 0;
	int ret = 0;
	int poll_faster = 0;

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	ret = tsens_get_temp(&tsens_dev, &temp);
	if (ret) {
		if (thermal_debug)
			pr_info("msm_thermal: Unable to read TSENS sensor %d\n",
				tsens_dev.sensor_num);
		goto reschedule;
	}

	if (thermal_debug)
		pr_info("msm_thermal: current CPU temperature %lu for sensor %d\n",temp, tsens_dev.sensor_num);

	if (!limit_init) {
		ret = msm_thermal_get_freq_table();
		if (ret)
			goto reschedule;
		else
			limit_init = 1;
	}
	
	/* max throttle exceeded - go direct to teh low step until it is under control */
	if (temp >= MAX_THROTTLE_TEMP) {
		poll_faster = 1;
		if (thermal_debug && throttle_on == true)
			pr_info("msm_thermal: throttling - CPU temp is %luC, max freq: %dMHz\n",temp, max_freq);
		limit_idx = min_freq_index;
		max_freq = table[limit_idx].frequency;
		if (throttle_on == false)
			pr_info("msm_thermal: throttling ON - threshold temp %dC reached, CPU temp is %luC\n", throttle_temp, temp);
		throttle_on = true;
		goto setmaxfreq;
	}

	//do_core_control(temp);

	/* temp is OK */
	if (temp < throttle_temp - msm_thermal_info.temp_hysteresis_degC) {
		if (throttle_on == true)
			pr_info("msm_thermal: throttling OFF, CPU temp is %luC\n", temp);
		throttle_on = false;
		if (limit_idx == limit_idx_high)
			goto reschedule;
		limit_idx = limit_idx_high;
		max_freq = table[limit_idx].frequency;
		goto setmaxfreq;
	}

	/* throttle exceeded - step down to the low step until it is under control */
	if (temp >= throttle_temp) {
		poll_faster = 1;
		if (thermal_debug && throttle_on == true)
			pr_info("msm_thermal: throttling - CPU temp is %luC, max freq: %dMHz\n",temp, max_freq);
		if (limit_idx == min_freq_index)
			goto reschedule;
		limit_idx -= msm_thermal_info.freq_step;
		if (limit_idx < min_freq_index)
			limit_idx = min_freq_index;
		max_freq = table[limit_idx].frequency;
		if (throttle_on == false)
			pr_info("msm_thermal: throttling ON - threshold temp %dC reached, CPU temp is %luC\n", throttle_temp, temp);
		throttle_on = true;
		goto setmaxfreq;
	}

	/* warning track - allow to go to max but poll faster */
	if (temp >= throttle_temp - msm_thermal_info.temp_hysteresis_degC) {
		poll_faster = 1;
		if (throttle_on == true)
			pr_info("msm_thermal: throttling OFF, CPU temp is %luC\n", temp);
		throttle_on = false;
		if (thermal_debug)
			pr_info("msm_thermal: cpu temp:%lu is nearing the threshold %d\n",temp, (throttle_temp - msm_thermal_info.temp_hysteresis_degC));
		if (limit_idx == limit_idx_high)
			goto reschedule;
		limit_idx = limit_idx_high;
		max_freq = table[limit_idx].frequency;
		goto setmaxfreq;
	}

setmaxfreq:

	/* Update new freq limits for all cpus */
	for_each_possible_cpu(cpu) {
		ret = update_cpu_max_freq(cpu, max_freq);
		if (ret) {
			if (thermal_debug)
				pr_info("msm_thermal: unable to limit cpu%d max freq to %d\n", cpu, max_freq);
		}
	}

reschedule:
	/* Reschedule next poll adjusting polling time (ms) on current situation */
	if (enabled) {
		if (temp > COOL_TEMP) {
			if (poll_faster) {
				if (thermal_debug)
					pr_info("msm_thermal: throttle temp is near, polling at %dms\n",msm_thermal_info.poll_ms - HOT_TEMP_OFFSET_MS);
				schedule_delayed_work(&check_temp_work,
						msecs_to_jiffies(msm_thermal_info.poll_ms - HOT_TEMP_OFFSET_MS));
			} else {
				if (thermal_debug)
					pr_info("msm_thermal: CPU temp is fine, polling at %dms\n",msm_thermal_info.poll_ms);
				schedule_delayed_work(&check_temp_work,
						msecs_to_jiffies(msm_thermal_info.poll_ms));
			}
		} else {
			if (thermal_debug)
				pr_info("msm_thermal: CPU temp cool, polling at %dms\n",msm_thermal_info.poll_ms + COOL_TEMP_OFFSET_MS);
			schedule_delayed_work(&check_temp_work,
					msecs_to_jiffies(msm_thermal_info.poll_ms + COOL_TEMP_OFFSET_MS));
		}
	}
}

static int __cpuinit msm_thermal_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;

	if (action == CPU_UP_PREPARE || action == CPU_UP_PREPARE_FROZEN) {
		if (core_control_enabled &&
			(msm_thermal_info.core_control_mask & BIT(cpu)) &&
			(cpus_offlined & BIT(cpu))) {
			pr_info(
			"%s: Preventing cpu%d from coming online.\n",
				KBUILD_MODNAME, cpu);
			return NOTIFY_BAD;
		}
	}


	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_thermal_cpu_notifier = {
	.notifier_call = msm_thermal_cpu_callback,
};

/**
 * We will reset the cpu frequencies limits here. The core online/offline
 * status will be carried over to the process stopping the msm_thermal, as
 * we dont want to online a core and bring in the thermal issues.
 */
static void __cpuinit disable_msm_thermal(void)
{
	int cpu = 0;

	/* make sure check_temp is no longer running */
	cancel_delayed_work(&check_temp_work);
	flush_scheduled_work();

	if (limited_max_freq == MSM_CPUFREQ_NO_LIMIT)
		return;

	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, MSM_CPUFREQ_NO_LIMIT);
	}
}

static int __cpuinit set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	if (!enabled)
		disable_msm_thermal();
	else
		pr_info("%s: no action for enabled = %d\n",
				KBUILD_MODNAME, enabled);

	pr_info("%s: enabled = %d\n", KBUILD_MODNAME, enabled);

	return ret;
}

static int set_debug(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	pr_info("msm_thermal: debug = %d\n", thermal_debug);

	return ret;
}

static int set_throttle_temp(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	long num;

	if (!val)
		return -EINVAL;

	ret = strict_strtol(val, 0, &num);
	if (ret == -EINVAL || num > MAX_THROTTLE_TEMP || num < COOL_TEMP)
		return -EINVAL;

	ret = param_set_int(val, kp);

	pr_info("msm_thermal: throttle_temp = %d\n", throttle_temp);

	return ret;
}

static int set_min_freq_index(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	long num;

	if (!val)
		return -EINVAL;

	ret = strict_strtol(val, 0, &num);
	if (ret == -EINVAL || num > 8 || num < 4)
		return -EINVAL;

	ret = param_set_int(val, kp);

	pr_info("msm_thermal: min_freq_index = %d\n", min_freq_index);

	return ret;
}

static struct kernel_param_ops module_ops_enabled = {
	.set = set_enabled,
	.get = param_get_bool,
};

static struct kernel_param_ops module_ops_debug = {
	.set = set_debug,
	.get = param_get_bool,
};

static struct kernel_param_ops module_ops_thermal_temp = {
	.set = set_throttle_temp,
	.get = param_get_uint,
};

static struct kernel_param_ops module_ops_min_freq_index = {
	.set = set_min_freq_index,
	.get = param_get_uint,
};

module_param_cb(enabled, &module_ops_enabled, &enabled, 0775);
MODULE_PARM_DESC(enabled, "msm_thermal enforce limit on cpu (Y/N)");

module_param_cb(thermal_debug, &module_ops_debug, &thermal_debug, 0775);
MODULE_PARM_DESC(enabled, "msm_thermal debug to kernel log (Y/N)");

module_param_cb(throttle_temp, &module_ops_thermal_temp, &throttle_temp, 0775);
MODULE_PARM_DESC(throttle_temp, "msm_thermal throttle temperature (C)");

module_param_cb(min_freq_index, &module_ops_min_freq_index, &min_freq_index, 0775);
MODULE_PARM_DESC(min_freq_index, "msm_thermal minimum throttle frequency index");


/* Call with core_control_mutex locked */
static int __cpuinit update_offline_cores(int val)
{
	int cpu = 0;
	int ret = 0;

	cpus_offlined = msm_thermal_info.core_control_mask & val;
	if (!core_control_enabled)
		return 0;

	for_each_possible_cpu(cpu) {
		if (!(cpus_offlined & BIT(cpu)))
		       continue;
		if (!cpu_online(cpu))
			continue;
		ret = cpu_down(cpu);
		if (ret)
			pr_err("%s: Unable to offline cpu%d\n",
				KBUILD_MODNAME, cpu);
	}
	return ret;
}

static ssize_t show_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", core_control_enabled);
}

static ssize_t __cpuinit store_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	mutex_lock(&core_control_mutex);
	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("%s: Invalid input %s\n", KBUILD_MODNAME, buf);
		goto done_store_cc;
	}

	if (core_control_enabled == !!val)
		goto done_store_cc;

	core_control_enabled = !!val;
	if (core_control_enabled) {
		pr_info("%s: Core control enabled\n", KBUILD_MODNAME);
		register_cpu_notifier(&msm_thermal_cpu_notifier);
		update_offline_cores(cpus_offlined);
	} else {
		pr_info("%s: Core control disabled\n", KBUILD_MODNAME);
		unregister_cpu_notifier(&msm_thermal_cpu_notifier);
	}

done_store_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static ssize_t show_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpus_offlined);
}

static ssize_t __cpuinit store_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	uint32_t val = 0;

	mutex_lock(&core_control_mutex);
	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("%s: Invalid input %s\n", KBUILD_MODNAME, buf);
		goto done_cc;
	}

	if (enabled) {
		pr_err("%s: Ignoring request; polling thread is enabled.\n",
				KBUILD_MODNAME);
		goto done_cc;
	}

	if (cpus_offlined == val)
		goto done_cc;

	update_offline_cores(val);
done_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static __cpuinitdata struct kobj_attribute cc_enabled_attr =
__ATTR(enabled, 0644, show_cc_enabled, store_cc_enabled);

static __cpuinitdata struct kobj_attribute cpus_offlined_attr =
__ATTR(cpus_offlined, 0644, show_cpus_offlined, store_cpus_offlined);

static __cpuinitdata struct attribute *cc_attrs[] = {
	&cc_enabled_attr.attr,
	&cpus_offlined_attr.attr,
	NULL,
};

static __cpuinitdata struct attribute_group cc_attr_group = {
	.attrs = cc_attrs,
};

static __init int msm_thermal_add_cc_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *cc_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module\n",
			KBUILD_MODNAME);
		ret = -ENOENT;
		goto done_cc_nodes;
	}

	cc_kobj = kobject_create_and_add("core_control", module_kobj);
	if (!cc_kobj) {
		pr_err("%s: cannot create core control kobj\n",
				KBUILD_MODNAME);
		ret = -ENOMEM;
		goto done_cc_nodes;
	}

	ret = sysfs_create_group(cc_kobj, &cc_attr_group);
	if (ret) {
		pr_err("%s: cannot create group\n", KBUILD_MODNAME);
		goto done_cc_nodes;
	}

	return 0;

done_cc_nodes:
	if (cc_kobj)
		kobject_del(cc_kobj);
	return ret;
}

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;

	BUG_ON(!pdata);
	BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	enabled = 1;
	core_control_enabled = 1;
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	schedule_delayed_work(&check_temp_work, 0);

	register_cpu_notifier(&msm_thermal_cpu_notifier);

	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	memset(&data, 0, sizeof(struct msm_thermal_data));
	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;
	WARN_ON(data.sensor_id >= TSENS_MAX_SENSORS);

	key = "qcom,poll-ms";
	ret = of_property_read_u32(node, key, &data.poll_ms);
	if (ret)
		goto fail;

	key = "qcom,limit-temp";
	ret = of_property_read_u32(node, key, &data.limit_temp_degC);
	if (ret)
		goto fail;

	key = "qcom,temp-hysteresis";
	ret = of_property_read_u32(node, key, &data.temp_hysteresis_degC);
	if (ret)
		goto fail;

	key = "qcom,freq-step";
	ret = of_property_read_u32(node, key, &data.freq_step);
	if (ret)
		goto fail;

	key = "qcom,core-limit-temp";
	ret = of_property_read_u32(node, key, &data.core_limit_temp_degC);
	if (ret)
		goto fail;

	key = "qcom,core-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data.core_temp_hysteresis_degC);
	if (ret)
		goto fail;

	key = "qcom,core-control-mask";
	ret = of_property_read_u32(node, key, &data.core_control_mask);
	if (ret)
		goto fail;

fail:
	if (ret)
		pr_err("%s: Failed reading node=%s, key=%s\n",
		       __func__, node->full_name, key);
	else
		ret = msm_thermal_init(&data);

	return ret;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

int __init msm_thermal_late_init(void)
{
	return msm_thermal_add_cc_nodes();
}
module_init(msm_thermal_late_init);
