FROM ubuntu:22.04 as ci_image

# set up global variables
ENV DEBIAN_FRONTEND='noninteractive' \
    TZ='Europe/Moscow' \
    USER='qt-docker' \
    UID=1000 \
    GID=1000

# install packages for QT5 developmewnt
RUN \
    set -eux && \
    apt-get update && \
    apt-get -y upgrade && \
    apt-get -y install \
    bash zsh g++ build-essential ccache clang clang-tidy cmake cppcheck curl gdb git gnupg gperf iputils-ping \
    libboost-all-dev libfcgi-dev libgfortran5 libgl1-mesa-dev libjemalloc-dev libjemalloc2 libmlpack-dev  \
    libtbb-dev libssl-dev libyaml-cpp-dev linux-tools-generic nano nasm ninja-build openssh-server openssh-client\
    openssl pkg-config python3 spawn-fcgi sudo tini unzip valgrind wget zip texinfo gcc-multilib chrpath \
    socat cpio xz-utils debianutils libegl1-mesa patch perl tar rsync bc libelf-dev libssl-dev libsdl1.2-dev \
    mesa-common-dev whois software-properties-common libx11-xcb-dev libxcb-dri3-dev libxcb-icccm4-dev libxcb-image0-dev \
    libxcb-keysyms1-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-render0-dev libxcb-shape0-dev libxcb-sync-dev \
    libxcb-util-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-xkb-dev xorg-dev libconfuse-dev libnl-3-dev \
    libnl-route-3-dev libncurses-dev dh-autoreconf freeglut3 freeglut3-dev libglfw3-dev \
    graphviz qtbase5-dev libqt5opengl5-dev qtdeclarative5-dev libqt5charts5-dev qmlscene qml-module-* xdot

# set up user
RUN groupadd -g ${GID} ${USER} && useradd --no-log-init --uid ${UID} --gid ${GID} --create-home ${USER}  \
    --shell /bin/bash && echo "${USER} ALL=(root) NOPASSWD:ALL" >> "/etc/sudoers.d/${USER}"

USER ${UID}:${GID}
WORKDIR /home/${USER}

ENTRYPOINT ["/bin/bash", "-l", "-c"]

FROM ci_image as devel_image
# install and set up zsh
RUN sh -c "$(wget --no-hsts -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.3/zsh-in-docker.sh)" -- \
    -t 'edvardm' \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-history-substring-search

ENV DEBIAN_FRONTEND='teletype'

ENTRYPOINT ["/bin/zsh"]