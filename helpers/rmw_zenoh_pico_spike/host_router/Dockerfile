ARG ROUTER_IMAGE=ros2_ws-ros2_zenoh_router:latest
FROM rust:1.88.0-bookworm@sha256:4727898c104ecd2e22d780925832502faee9fe4e70581b8572af081370b315a0 AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true CARGO_BUILD_JOBS=6 RUSTUP_TOOLCHAIN=1.93.0

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

FROM builder AS baseline-builder
RUN cargo build --release --locked --features shared-memory,unstable,zenoh/transport_serial

FROM baseline-builder AS patched-builder
COPY zenoh-serial-accept.patch /build/zenoh-serial-accept.patch
RUN source_dir=$(find "$CARGO_HOME/git/checkouts" -maxdepth 3 -type d -name 2687c51) \
    && test -n "$source_dir" \
    && test "$(git -C "$source_dir" rev-parse HEAD)" = 2687c51352121f006e3a603ce07925a8ad0b295c \
    && git -C "$source_dir" apply --check /build/zenoh-serial-accept.patch \
    && git -C "$source_dir" apply /build/zenoh-serial-accept.patch \
    && git -C "$source_dir" diff --check \
    && cargo clean --release -p zenoh-link-serial \
    && cargo build --release --locked --features shared-memory,unstable,zenoh/transport_serial \
    && sha256sum target/release/libzenohc.so /build/zenoh-serial-accept.patch > /build/candidate.sha256

FROM ${ROUTER_IMAGE} AS rebuilt-baseline
COPY --from=baseline-builder /build/zenoh-c/target/release/libzenohc.so /opt/ros/lyrical/opt/zenoh_cpp_vendor/lib/libzenohc.so
COPY --from=baseline-builder /build/zenoh-c/include/ /opt/picoros-host-listener/include/

FROM ${ROUTER_IMAGE} AS candidate
COPY --from=patched-builder /build/zenoh-c/target/release/libzenohc.so /opt/ros/lyrical/opt/zenoh_cpp_vendor/lib/libzenohc.so
COPY --from=patched-builder /build/zenoh-c/include/ /opt/picoros-host-listener/include/
COPY --from=patched-builder /build/candidate.sha256 /opt/picoros-host-listener/candidate.sha256
LABEL org.relobot.experiment="serial-accept-reopen" \
    org.relobot.zenoh-revision="2687c51352121f006e3a603ce07925a8ad0b295c" \
    org.relobot.zenoh-c-revision="05bd370343b5161ca9269649b9a914c9c2dc4170"
