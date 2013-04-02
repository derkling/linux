/*
 * Copyright (C) 2013 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

extern struct cpumask hmp_fast_cpumask;
extern struct cpumask hmp_slow_cpumask;

void __init arch_get_hmp_domains(struct list_head *hmp_domains_list)
{
	struct hmp_domain *domain;

	/*
	 * Initialize hmp_domains
	 * Must be ordered with respect to compute capacity.
	 * Fastest domain at head of list.
	 */
	if (!cpumask_empty(hmp_slow_cpumask)) {
		domain = (struct hmp_domain *)
			kmalloc(sizeof(struct hmp_domain), GFP_KERNEL);
		cpumask_copy(&domain->cpus, &hmp_slow_cpumask);
		list_add(&domain->hmp_domains, hmp_domains_list);
	}

	domain = (struct hmp_domain *)kmalloc(sizeof(struct hmp_domain), GFP_KERNEL);
	cpumask_copy(&domain->cpus, &hmp_fast_cpu_mask);
	list_add(&domain->hmp_domains, hmp_domains_list);
}
