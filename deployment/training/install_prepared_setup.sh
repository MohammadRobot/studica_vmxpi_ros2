#!/usr/bin/env bash
# Apply only a previously provisioned offline root, without changing networking
# or SSH policy and without selecting or activating a production release.
set -euo pipefail
[[ ${EUID} -eq 0 ]] || { echo 'run as root' >&2; exit 1; }
setup_root=/home/vmx/studica-platform-setup-20260907
prepared_root=${setup_root}/prepared-root
training_root=/opt/studica/training-20260907
[[ -d ${prepared_root}/etc/studica && -d ${prepared_root}/var/lib/studica ]]
[[ ! -e /opt/studica/current && ! -L /opt/studica/current ]]
# Refuse to overwrite configuration, device identity, secrets or units.
for destination in /etc/studica "${training_root}" \
  /etc/systemd/journald.conf.d/studica.conf \
  /etc/systemd/system/studica-training-lidar.service; do
  [[ ! -e ${destination} && ! -L ${destination} ]] || exit 1
done
for unit in "${prepared_root}"/etc/systemd/system/studica*; do
  [[ ! -e /etc/systemd/system/${unit##*/} && ! -L /etc/systemd/system/${unit##*/} ]] || exit 1
done
for item in "${prepared_root}"/var/lib/studica/*; do
  [[ ! -e /var/lib/studica/${item##*/} && ! -L /var/lib/studica/${item##*/} ]] || exit 1
done
for account in studica studica-update; do
  if ! getent passwd "${account}" >/dev/null; then
    useradd --system --user-group --home-dir /var/lib/studica \
      --shell /usr/sbin/nologin "${account}"
  fi
done
usermod -a -G dialout,input,video studica
usermod -a -G studica studica-update
cp -a "${prepared_root}/etc/studica" /etc/studica
chown -R root:root /etc/studica
for unit in "${prepared_root}"/etc/systemd/system/studica*; do
  install -o root -g root -m 0644 "${unit}" /etc/systemd/system/
done
install -d -o root -g studica -m 0750 /var/lib/studica
for item in "${prepared_root}"/var/lib/studica/*; do
  cp -a "${item}" /var/lib/studica/
done
chown root:studica /var/lib/studica/*.json
for directory in maps pairing support secrets tls; do
  chown -R studica:studica "/var/lib/studica/${directory}"
done
chown -R studica-update:studica-update /var/lib/studica/updates
install -d -m 0755 /etc/systemd/journald.conf.d
install -o root -g root -m 0644 \
  "${prepared_root}/etc/systemd/journald.conf.d/studica.conf" \
  /etc/systemd/journald.conf.d/studica.conf
install -d -o root -g root -m 0755 "${training_root}"
install -o root -g root -m 0755 "${setup_root}/scripts/studica_sensor_runtime" "${training_root}/"
install -o root -g root -m 0644 \
  "${setup_root}/bringup/config/network/cyclonedds_sim.xml" "${training_root}/cyclonedds.xml"
install -o root -g root -m 0644 \
  "${setup_root}/deployment/training/studica-training-lidar.service" /etc/systemd/system/
systemctl daemon-reload
echo 'Installed platform configuration, identity and disabled production services.'
echo 'Network, hostname, SSH policy and production release pointers unchanged.'
echo 'Training LiDAR installed but not yet started or enabled; validate it first.'
