// SPDX-License-Identifier: GPL-2.0-only
/*
 * RISC-V RPMI Proxy (RPXY) Helper functions
 *
 * Copyright (C) 2023 Ventana Micro Systems Inc.
 */

#define pr_fmt(fmt) "riscv-rpxy: " fmt

#include <asm/sbi.h>
#include <linux/percpu.h>
#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/smp.h>

struct sbi_rpxy {
	void *shmem;
	phys_addr_t shmem_phys;
	bool active;
};

DEFINE_PER_CPU(struct sbi_rpxy, sbi_rpxy);

DEFINE_STATIC_KEY_FALSE(sbi_rpxy_available);
#define sbi_rpxy_available() \
	static_branch_unlikely(&sbi_rpxy_available)

int sbi_rpxy_srvgrp_probe(u32 transportid, u32 srvgrpid, unsigned long *val)
{
	struct sbiret sret;
	struct sbi_rpxy *rpxy;

	if (!sbi_rpxy_available())
		return -ENODEV;

	rpxy = this_cpu_ptr(&sbi_rpxy);

	get_cpu();
	sret = sbi_ecall(SBI_EXT_RPXY, SBI_EXT_RPXY_PROBE,
		  transportid, srvgrpid, 0, 0, 0, 0);
	if (val)
		*val = sret.value;
	put_cpu();

	return sbi_err_map_linux_errno(sret.error);
}
EXPORT_SYMBOL(sbi_rpxy_srvgrp_probe);

int sbi_rpxy_send_normal_message(u32 transportid, u32 srvgrpid, u8 srvid,
				 void *tx, unsigned long tx_msglen,
				 void *rx, unsigned long *rx_msglen)
{
	struct sbiret sret;
	struct sbi_rpxy *rpxy = this_cpu_ptr(&sbi_rpxy);

	if (!sbi_rpxy_available() || !rpxy->active)
		return -ENODEV;

	get_cpu();
	if (tx_msglen)
		memcpy(rpxy->shmem, tx, tx_msglen);

	/* Shared memory is copied with message data at 0x0 offset */
	sret = sbi_ecall(SBI_EXT_RPXY, SBI_EXT_RPXY_SEND_NORMAL_MSG,
			 transportid, srvgrpid, srvid, tx_msglen, 0, 0);

	if (!sret.error && rx) {
		memcpy(rx, rpxy->shmem, sret.value);
		if (rx_msglen)
			*rx_msglen = sret.value;
	}
	put_cpu();

	return sbi_err_map_linux_errno(sret.error);
}
EXPORT_SYMBOL(sbi_rpxy_send_normal_message);

int sbi_rpxy_send_posted_message(u32 transportid, u32 srvgrpid, u8 srvid,
				 void *tx, unsigned long tx_msglen)
{
	struct sbiret sret;
	struct sbi_rpxy *rpxy = this_cpu_ptr(&sbi_rpxy);

	if (!sbi_rpxy_available() || !rpxy->active)
		return -ENODEV;

	get_cpu();
	if (tx_msglen)
		memcpy(rpxy->shmem, tx, tx_msglen);

	/* Shared memory is copied with message data at 0x0 offset */
	sret = sbi_ecall(SBI_EXT_RPXY, SBI_EXT_RPXY_SEND_POSTED_MSG,
			 transportid, srvgrpid, srvid, tx_msglen, 0, 0);
	put_cpu();

	return sbi_err_map_linux_errno(sret.error);
}
EXPORT_SYMBOL(sbi_rpxy_send_posted_message);

int sbi_rpxy_get_notifications(u32 transportid, u32 srvgrpid,
			       void *rx, unsigned long *rx_msglen)
{
	struct sbiret sret;
	struct sbi_rpxy *rpxy = this_cpu_ptr(&sbi_rpxy);

	if (!sbi_rpxy_available() || !rpxy->active)
		return -ENODEV;

	get_cpu();
	sret = sbi_ecall(SBI_EXT_RPXY, SBI_EXT_RPXY_GET_NOTIFICATIONS,
			transportid, srvgrpid, 0, 0, 0, 0);

	if (!sret.error && rx) {
		memcpy(rx, rpxy->shmem, sret.value);
		if (rx_msglen)
			*rx_msglen = sret.value;
	}
	put_cpu();

	return sbi_err_map_linux_errno(sret.error);
}
EXPORT_SYMBOL(sbi_rpxy_get_notifications);

static int sbi_rpxy_exit(unsigned int cpu)
{
	struct sbi_rpxy *rpxy;

	if (!sbi_rpxy_available())
		return -ENODEV;

	rpxy = per_cpu_ptr(&sbi_rpxy, cpu);

	if (!rpxy->shmem)
		return -ENOMEM;

	free_pages((unsigned long)rpxy->shmem, get_order(PAGE_SIZE));
	rpxy->shmem = NULL;
	rpxy->shmem_phys = 0;
	rpxy->active = false;

	return 0;
}

static int sbi_rpxy_setup_shmem(unsigned int cpu)
{
	struct sbiret sret;
	struct page *shmem_page;
	struct sbi_rpxy *rpxy;

	if (!sbi_rpxy_available())
		return -ENODEV;

	rpxy = per_cpu_ptr(&sbi_rpxy, cpu);
	if (rpxy->active)
		return -EINVAL;

	shmem_page = alloc_pages(GFP_KERNEL | __GFP_ZERO,
				 get_order(PAGE_SIZE));
	if (!shmem_page) {
		sbi_rpxy_exit(cpu);
		pr_err("Shared memory setup failed for cpu-%d\n", cpu);
		return -ENOMEM;
	}

	rpxy->shmem = page_to_virt(shmem_page);
	rpxy->shmem_phys = page_to_phys(shmem_page);

	sret = sbi_ecall(SBI_EXT_RPXY, SBI_EXT_RPXY_SETUP_SHMEM,
			PAGE_SIZE, rpxy->shmem_phys, 0, 0, 0, 0);
	if (sret.error) {
		sbi_rpxy_exit(cpu);
		return sbi_err_map_linux_errno(sret.error);
	}

	rpxy->active = true;

	return 0;
}

static int __init sbi_rpxy_init(void)
{
	if ((sbi_spec_version < sbi_mk_version(1, 0)) ||
		sbi_probe_extension(SBI_EXT_RPXY) <= 0) {
		return -ENODEV;
	}

	static_branch_enable(&sbi_rpxy_available);
	pr_info("SBI RPXY extension detected\n");

	/*
	 * Setup CPUHP notifier to setup shared
	 * memory on all CPUs
	 */
	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
			  "riscv/rpxy-sbi:cpu-shmem-init",
			  sbi_rpxy_setup_shmem,
			  sbi_rpxy_exit);

	return 0;
}

arch_initcall(sbi_rpxy_init);
