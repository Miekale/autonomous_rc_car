WIFI = "celines_iphone_9"
WIFIPS = "oceanshrimp"

sleep 5

nmcli device disconnect wlan0

sleep 1

nmcli device wifi connect $WIFI password $WIFIPS