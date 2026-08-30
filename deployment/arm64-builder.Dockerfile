# syntax=docker/dockerfile:1.7

ARG UBUNTU_BASE_IMAGE=ubuntu:jammy-20260810@sha256:2edbbc5dc405e9612ba3584ce95480277e3eb374407b5505fe26f17df77c7dbc

FROM --platform=linux/arm64 ${UBUNTU_BASE_IMAGE} AS ros-apt-base

ARG TARGETARCH

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

RUN test "${TARGETARCH}" = "arm64"
RUN apt-get -o Acquire::Retries=3 update \
    && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      ca-certificates \
      python3 \
    && rm -rf /var/lib/apt/lists/*

ADD --checksum=sha256:767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433 \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb \
  /tmp/ros2-apt-source.deb
RUN dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb


FROM ros-apt-base AS build-env

COPY dependencies/apt/development-core.txt /opt/studica/development-core.txt
COPY deployment/ros-package-schema-catalog.xml \
  /opt/studica/ros-schema/catalog.xml
ADD --checksum=sha256:f096a197ed6d7878984bb2501a55f7f1bd4895d254399fb7857e154bfb644f41 \
  https://raw.githubusercontent.com/ros-infrastructure/rep/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/xsd/package_format3.xsd \
  /opt/studica/ros-schema/package_format3.xsd
ADD --checksum=sha256:941ea8645344f3c4b7b9d7e68799898309d65a18225fa9cbef4169d95d1a3211 \
  https://raw.githubusercontent.com/ros-infrastructure/rep/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/xsd/package_common.xsd \
  /opt/studica/ros-schema/package_common.xsd
RUN apt-get -o Acquire::Retries=3 update \
    && sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' \
      /opt/studica/development-core.txt > /tmp/development-packages.txt \
    && xargs -r apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      < /tmp/development-packages.txt \
    && rm -f /tmp/development-packages.txt

COPY dependencies/hardware.repos /opt/studica/hardware.repos
COPY package.xml /opt/studica/studica_vmxpi_ros2.package.xml
RUN rosdep init \
    && rosdep update --rosdistro humble \
    && mkdir -p /opt/studica/rosdep-cache \
    && cp -a /root/.ros/rosdep/sources.cache/. \
      /opt/studica/rosdep-cache/ \
    && mkdir -p /tmp/studica-rosdep/src/studica_vmxpi_ros2 \
    && vcs import --recursive /tmp/studica-rosdep/src \
      < /opt/studica/hardware.repos \
    && cp /opt/studica/studica_vmxpi_ros2.package.xml \
      /tmp/studica-rosdep/src/studica_vmxpi_ros2/package.xml \
    && rosdep --sources-cache-dir /opt/studica/rosdep-cache install \
      --from-paths /tmp/studica-rosdep/src \
      --ignore-src \
      --rosdistro humble \
      --skip-keys "gz_ros2_control ros_gz_bridge ros_gz_sim" \
      -y \
    && chmod -R a+rX /opt/studica/rosdep-cache \
    && rm -rf /tmp/studica-rosdep /var/lib/apt/lists/* /root/.ros

COPY deployment/build_arm64_release_in_container.sh \
  /usr/local/bin/build-studica-arm64-release
COPY deployment/prepare_arm64_release_sources.sh \
  /usr/local/bin/prepare-studica-arm64-sources
RUN chmod 0644 \
      /opt/studica/ros-schema/package_common.xsd \
      /opt/studica/ros-schema/package_format3.xsd \
    && chmod 0755 \
      /usr/local/bin/build-studica-arm64-release \
      /usr/local/bin/prepare-studica-arm64-sources \
    && mkdir -p \
      /inputs \
      /output \
      /prepared \
      /source \
      /workspace \
      /usr/local/include/vmxpi \
      /usr/local/lib/vmxpi

ENTRYPOINT ["/usr/local/bin/build-studica-arm64-release"]


FROM ros-apt-base AS runtime-inventory

COPY deployment/vmxpi-runtime-packages-v1.json \
  /opt/studica/vmxpi-runtime-packages-v1.json
RUN python3 -c 'import json; p=json.load(open("/opt/studica/vmxpi-runtime-packages-v1.json")); names=set(); [names.update(group) for group in p["required_apt_packages"].values()]; [names.update(feature["apt_packages"]) for feature in p["features"].values() if feature["enabled"]]; print("\n".join(sorted(names)))' \
      > /tmp/runtime-packages.txt \
    && apt-get -o Acquire::Retries=3 update \
    && xargs -r apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      < /tmp/runtime-packages.txt \
    && rm -rf /var/lib/apt/lists/* /tmp/runtime-packages.txt

CMD ["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\t${Architecture}\n"]
