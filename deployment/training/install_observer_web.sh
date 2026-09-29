#!/usr/bin/env bash
# Narrow one-time install: observation-only web, no production activation.
set -euo pipefail
umask 077
[[ ${EUID} -eq 0 ]] || exit 1
setup_root=/home/vmx/studica-platform-setup-20260907
training_root=/opt/studica/training-20260907
[[ -d ${training_root} && ! -e /opt/studica/current && ! -L /opt/studica/current ]]
for destination in "${training_root}/python" "${training_root}/web" \
  "${training_root}/studica_observer_web_runtime" \
  /etc/systemd/system/studica-training-web.service \
  /var/lib/studica/tls/observer.key /var/lib/studica/tls/observer.crt; do
  [[ ! -e ${destination} && ! -L ${destination} ]] || exit 1
done
install -d -o root -g root -m 0755 "${training_root}/python/studica_robot_platform"
for module in __init__ model map_registry pairing orchestrator_client web_server sensor_observer; do
  install -o root -g root -m 0644 "${setup_root}/studica_robot_platform/${module}.py" \
    "${training_root}/python/studica_robot_platform/"
done
install -d -o root -g root -m 0755 "${training_root}/web/assets"
install -o root -g root -m 0644 "${setup_root}/web/index.html" "${training_root}/web/"
install -o root -g root -m 0644 "${setup_root}/web/assets/app.css" "${setup_root}/web/assets/app.js" \
  "${training_root}/web/assets/"
install -o root -g root -m 0755 "${setup_root}/deployment/training/studica_observer_web_runtime" "${training_root}/"
openssl req -new -newkey rsa:2048 -nodes -subj '/CN=Studica sensor observer' \
  -keyout /var/lib/studica/tls/observer.key \
  -out /var/lib/studica/tls/observer.csr >/dev/null 2>&1
openssl x509 -req -in /var/lib/studica/tls/observer.csr \
  -CA /var/lib/studica/tls/robot.crt -CAkey /var/lib/studica/tls/robot.key \
  -set_serial "0x$(openssl rand -hex 16)" -days 365 -sha256 \
  -extfile "${setup_root}/deployment/training/observer-tls.ext" \
  -out /var/lib/studica/tls/observer.crt
chown studica:studica /var/lib/studica/tls/observer.key /var/lib/studica/tls/observer.crt
chmod 0600 /var/lib/studica/tls/observer.key
chmod 0644 /var/lib/studica/tls/observer.crt
openssl verify -CAfile /var/lib/studica/tls/robot.crt /var/lib/studica/tls/observer.crt
install -o root -g root -m 0644 \
  "${setup_root}/deployment/training/studica-training-web.service" /etc/systemd/system/
systemd-analyze verify /etc/systemd/system/studica-training-web.service
systemctl daemon-reload
echo 'Observation-only web installed, not started or enabled. Production services unchanged.'
