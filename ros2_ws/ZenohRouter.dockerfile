FROM rust:1.88.0-bookworm@sha256:af306cfa71d987911a781c37b59d7d67d934f49684058f96cf72079c3626bfe0 AS serial-builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG CARGO_BUILD_JOBS=2
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true RUSTUP_TOOLCHAIN=1.93.0

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake libclang-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rustup toolchain install 1.93.0 --profile minimal

WORKDIR /build/zenoh-c
RUN git init && git remote add origin https://github.com/eclipse-zenoh/zenoh-c.git \
    && git fetch --depth=1 origin 05bd370343b5161ca9269649b9a914c9c2dc4170 \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = 05bd370343b5161ca9269649b9a914c9c2dc4170 \
    && cargo fetch --locked

COPY helpers/rmw_zenoh_pico_spike/host_router/zenoh-serial-accept.patch /build/zenoh-serial-accept.patch
RUN source_dir=$(find "$CARGO_HOME/git/checkouts" -maxdepth 3 -type d -name 2687c51) \
    && test -n "$source_dir" \
    && test "$(git -C "$source_dir" rev-parse HEAD)" = 2687c51352121f006e3a603ce07925a8ad0b295c \
    && git -C "$source_dir" apply --check /build/zenoh-serial-accept.patch \
    && git -C "$source_dir" apply /build/zenoh-serial-accept.patch \
    && git -C "$source_dir" diff --check \
    && cargo build --release --locked --features shared-memory,unstable,zenoh/transport_serial \
    && sha256sum target/release/libzenohc.so /build/zenoh-serial-accept.patch > /build/candidate.sha256

FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rmw-zenoh-cpp \
    && vendor_version="$(dpkg-query -W -f='${Version}' ros-lyrical-zenoh-cpp-vendor)" \
    && test "${vendor_version%%-*}" = 0.10.6 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=serial-builder /build/zenoh-c/target/release/libzenohc.so /opt/ros/lyrical/opt/zenoh_cpp_vendor/lib/libzenohc.so
COPY --from=serial-builder /build/zenoh-c/include/ /opt/picoros-host-listener/include/
COPY --from=serial-builder /build/candidate.sha256 /opt/picoros-host-listener/candidate.sha256
COPY ros2_ws/start_zenoh_router.sh /usr/local/bin/start_zenoh_router.sh
COPY ros2_ws/zenoh_devices.py /usr/local/bin/zenoh_devices.py

LABEL org.relobot.zenoh-revision="2687c51352121f006e3a603ce07925a8ad0b295c" \
    org.relobot.zenoh-c-revision="05bd370343b5161ca9269649b9a914c9c2dc4170"

CMD ["bash", "/usr/local/bin/start_zenoh_router.sh"]
