FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ARM_TOOLCHAIN=15.2.rel1

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        clang-format \
        cmake \
        cppcheck \
        curl \
        gdb-multiarch \
        git \
        ninja-build \
        openocd \
        python3 \
        python3-serial \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL \
        "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN}-x86_64-arm-none-eabi.tar.xz" \
        | tar -xJ -C /opt \
    && mv "/opt/arm-gnu-toolchain-${ARM_TOOLCHAIN}-x86_64-arm-none-eabi" /opt/arm-none-eabi

ENV PATH=/opt/arm-none-eabi/bin:${PATH}
