# 直接配置
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
# 永久配置
sudo nano /etc/netplan/01-netcfg.yaml  
```
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.102/24]
      gateway4: 192.168.1.1   # 可选
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```
sudo netplan apply
