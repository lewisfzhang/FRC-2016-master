#!/bin/sh

cd ~/

# copy installation files into their expected places
echo "[mosquitto] Copy files..."
mkdir -p /usr/local/lib/
cp libzlib.so libwebsockets.so /usr/local/lib/

mkdir -p /usr/local/bin/
cp mosquitto /usr/local/bin/

mkdir -p /usr/local/conf/
cp mosquitto.roborio.conf /usr/local/conf/

cp mosquitto.init.sh /etc/init.d/

echo "[mosquitto] Configure init..."
# Create the mosquitto RC info
cd /etc/init.d/
update-rc.d mosquitto.init.sh defaults

ldconfig

echo "[mosquitto] Finished"
