Ubuntu Server uses Netplan to manage its connections.

To create an access point using Netplan, you can do the following:
#### 1. Install Network Manager

```bash
sudo apt update
sudo apt install network-manager
```

#### 2. Disable cloud-init

```bash
sudo bash -c "echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"
```

####  3. Create a Netplan configuration

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
Then add the following configuration:

```bash
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "Raspberry":
          password: "12345678"
          mode: ap
        "Additional_WIFI_Name_1":
          password: "12345678"
        "Additional_WIFI_Name_2":
          password: "12345678"
```
You can change the access point name "Raspberry" and the password to your liking.

Then save the file using CTRL+X.
####  4. Apply the Netplan configuration

Finally, use use the following commands to apply your new configuration:

```bash
sudo netplan generate
sudo netplan apply
sudo reboot
```
A new wireless access point should be created. It has DHCP and DNS enabled by default, and if the Pi has internet access over Ethernet, it'll be shared over the WiFi hotspot as well.

#### 5. To enable/disable Wifi 

Run this command:

The command template is look like this:
```bash
sudo nmcli c up/down your_wifi_name
```

This command is used to activate a hotspot connection:
```bash
sudo nmcli c up netplan-wlan0-Raspberry
```

To switch to others WiFi connection - run this
```bash
sudo nmcli c up Additional_WIFI_Name_1
```

```bash
sudo nmcli c down Additional_WIFI_Name_1
```

#### 6. To display a list of visible Wi-Fi networks:

```bash
nmcli d wifi list
```
```bash
nmcli connection show netplan-wlan0-Raspberry
```

#### 7. Bash script

The connections control can be wrapped to the bash script. The example are provided in the `bash` folder.