# setup MQTT on fedora (NOT FINISHED)

installation

```shell
sudo dnf install mosquitto
```

config

```shell
cd /etc/mosquitto
sudo mkdir -p /etc/mosquitto/conf.d
sudo vim /etc/mosquitto/mosquitto.conf
echo "include_dir /etc/mosquitto/conf.d/" | sudo tee -a /etc/mosquitto/mosquitto.conf
sudo vim /etc/mosquitto/mosquitto.conf
sudo tee /etc/mosquitto/conf.d/01-production.conf << EOF
# Local unencrypted listener
listener 1883 0.0.0.0
allow_anonymous true

# External encrypted listener
listener 8883 0.0.0.0
cafile /etc/mosquitto/certs/ca.crt
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
tls_version tlsv1.2
require_certificate false
allow_anonymous true
password_file /etc/mosquitto/passwd

# General settings
max_connections -1
max_inflight_messages 100
max_queued_messages 1000
autosave_interval 1800
EOF
sudo chown mosquitto:mosquitto -R certs/

# add a user
sudo mosquitto_passwd /etc/mosquitto/passwd iot_device_user
```

generate certificates

```shell
openssl genrsa -out ca.key 2048

# here I set "Common Name" to m80q
openssl req -new -x509 -days 3650 -key ca.key -out ca.crt

openssl genrsa -out server.key 2048

# here I set "Common Name" to localhost
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 3650
```

copy certs to $HOME

```shell
mkdir -p ~/mqtt-certs
sudo cp /etc/mosquitto/certs/{ca.crt,client.crt,client.key} ~/mqtt-certs/
sudo chown $USER:$USER ~/mqtt-certs/*
```

check

```shell
tmux

# left
mosquitto_sub -h m80q -p 8883 --cafile ~/mqtt-certs/ca.crt --cert ~/mqtt-certs/client.crt --key ~/mqtt-certs/client.key -u iot_device_user -P thepassword -t test -d

# right
mosquitto_pub -h m80q -p 8883 \
  --cafile ~/mqtt-certs/ca.crt \
  --cert ~/mqtt-certs/client.crt \
  --key ~/mqtt-certs/client.key \
  -u iot_device_user -P thepassword \
  -t test -m "hello tls" -d
```
