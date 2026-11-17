FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ARM_TOOLCHAIN=15.2.rel1
ARG ARM_TOOLCHAIN_SHA256=597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16
ARG CLANG_FORMAT_VERSION=23.1.5

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        cppcheck \
        curl \
        gdb-multiarch \
        git \
        ninja-build \
        openocd \
        pipx \
        python3 \
        python3-serial \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL -o /tmp/arm.tar.xz \
        "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN}-x86_64-arm-none-eabi.tar.xz" \
    && echo "${ARM_TOOLCHAIN_SHA256}  /tmp/arm.tar.xz" | sha256sum -c - \
    && mkdir -p /opt/arm-none-eabi \
    && tar -xJf /tmp/arm.tar.xz -C /opt/arm-none-eabi --strip-components=1 \
    && rm /tmp/arm.tar.xz

RUN PIPX_HOME=/opt/pipx PIPX_BIN_DIR=/usr/local/bin pipx install "clang-format==${CLANG_FORMAT_VERSION}"

ENV PATH=/opt/arm-none-eabi/bin:${PATH}
