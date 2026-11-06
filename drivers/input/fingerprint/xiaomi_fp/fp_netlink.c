/*
 * Copyright (C) 2022-2023, Xiaomi, Inc.
 * All Rights Reserved.
 */
#include "fp_driver.h"

#define MAX_NL_MSG_LEN 16

static struct sock *nl_sk;
static u32 peer_portid;
static DEFINE_SPINLOCK(netlink_lock);

void netlink_send(struct fp_device *fp_dev, const int command)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	struct sock *sk;
	unsigned long flags;
	u32 portid;
	int ret;

	spin_lock_irqsave(&netlink_lock, flags);
	sk = nl_sk;
	portid = peer_portid;
	if (sk)
		sock_hold(sk);
	spin_unlock_irqrestore(&netlink_lock, flags);
	if (!sk)
		return;
	if (!portid)
		goto put;
	skb = nlmsg_new(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (!skb)
		goto put;
	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		goto put;
	}
	/* Preserve stock's 16-byte payload and one-byte event command. */
	memset(nlmsg_data(nlh), 0, MAX_NL_MSG_LEN);
	*(u8 *)nlmsg_data(nlh) = command;
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	ret = netlink_unicast(sk, skb, portid, MSG_DONTWAIT);
	if (ret < 0)
		pr_debug("netlink event %d failed: %d\n", command, ret);
put:
	sock_put(sk);
}

void netlink_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	unsigned long flags;
	u32 portid;

	if (!skb || skb->len < nlmsg_total_size(0))
		return;
	nlh = nlmsg_hdr(skb);
	if (!nlmsg_ok(nlh, skb->len))
		return;
	portid = NETLINK_CB(skb).portid;
	if (!portid)
		return;
	spin_lock_irqsave(&netlink_lock, flags);
	peer_portid = portid;
	spin_unlock_irqrestore(&netlink_lock, flags);
	pr_debug("registered fingerprint netlink port %u\n", portid);
}

int netlink_init(struct fp_device *fp_dev)
{
	struct netlink_kernel_cfg cfg = { .input = netlink_recv };
	struct sock *sk;
	unsigned long flags;

	/* Probe/ioctl/remove serialize socket ownership with fp_state_lock. */
	if (READ_ONCE(nl_sk))
		return 0;
	sk = netlink_kernel_create(&init_net, fp_dev->netlink_num, &cfg);
	if (!sk)
		return -ENOMEM;
	spin_lock_irqsave(&netlink_lock, flags);
	peer_portid = 0;
	nl_sk = sk;
	spin_unlock_irqrestore(&netlink_lock, flags);
	return 0;
}

int netlink_destroy(struct fp_device *fp_dev)
{
	struct sock *sk;
	unsigned long flags;

	spin_lock_irqsave(&netlink_lock, flags);
	sk = nl_sk;
	nl_sk = NULL;
	peer_portid = 0;
	spin_unlock_irqrestore(&netlink_lock, flags);
	if (sk)
		netlink_kernel_release(sk);
	return 0;
}
