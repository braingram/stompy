---
layout: default
title: Network
---

IP Addresses:

- fl: 10.0.1.10
- ml: 10.0.1.11
- rl: 10.0.1.12
- rr: 10.0.1.13
- mr: 10.0.1.14
- fr: 10.0.1.15
- head: 10.0.1.20


Network configuration:

- set hostname in: /etc/hostname
- set localhost in: /etc/hosts
- setup network interface in: /etc/dhcpd.conf (below)

```
interface eth0

static ip_address=10.0.1.15/24
static routers=10.0.1.1
static domain_name_servers=10.0.1.1
```
