FROM shadowrobot/build-tools:xenial-kinetic

MAINTAINER "Andriy Petlovanyy"

LABEL Description="This is example Arduino ROS Kinetic image with Cloud 9 Web IDE" Vendor="Andriy Petlovanyy" Version="1.0"

ARG first_aurora_branch

ENV first_aurora_branch ${first_aurora_branch:-"master"}

ENV first_aurora_script "https://raw.githubusercontent.com/ros-ukraine/aurora/$first_aurora_branch/bin/run-ansible.sh"

ENV remote_deploy_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$toolset_branch/ansible/deploy.sh"

ENV LOCAL_USER_ID 1000

ENV MY_USERNAME user

ENV DISPLAY :1.0

ENV QT_X11_NO_MITSHM 1

USER root

RUN set -x && \
    \
    echo "Installing and compiling source code" && \
    wget -O /tmp/oneliner_deploy "$( echo "$remote_deploy_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner_deploy && \
    gosu $MY_USERNAME /tmp/oneliner_deploy -o AndriyPt -r rosserial-arduino-examples -b kinetic-devel -v kinetic && \
    \
    echo "Clean up" && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /root/.ansible /root/.gitconfig /root/.cache

USER user

COPY startup.sh /home/user/

RUN set -x && \
    \
    echo "Sourcing workspace in .bashrc" && \
    echo 'source /home/$MY_USERNAME/workspace/rosserial-arduino-examples/base/devel/setup.bash' >> /home/$MY_USERNAME/.bashrc && \
    \
    echo "Downloading one-liner" && \
    wget -O /tmp/oneliner "$( echo "$first_aurora_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    \
    echo "Installing IDEs and AWS CLI" && \
    /tmp/oneliner install_software --debug-branch $first_aurora_branch software=[cloud9,gzweb] && \
    \
    echo "Generating models for visualization" && \
    pushd /home/user/gzweb && \
    npm run deploy --- -m local && \
    popd && \
    \
    echo "Clean up" && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /root/.ansible /root/.gitconfig /root/.cache

USER root

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

EXPOSE 8080 8181 8282

CMD  ["/home/user/startup.sh"]
