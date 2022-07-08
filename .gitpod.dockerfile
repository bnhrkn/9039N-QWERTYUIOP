FROM gitpod/workspace-full


ADD ./work

RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz | tar -xJ

ENV PATH "/workspace/gcc-arm*arm-none-eabi/bin:$PATH"