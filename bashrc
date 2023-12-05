#
# ~/.bashrc
#

# If not running interactively, don't do anything
[[ $- != *i* ]] && return

alias ls='ls --color=auto'
alias ll='ls -l'
alias rm='rm -i'
alias mv='mv -i'
alias cp='cp -i'
alias grep='grep --color=auto'
export EDITOR='vim'
export LESS="-iMSx4 -FX -SR"
alias less="less ${LESS}"
export PAGER="less ${LESS}"
export QT_X11_NO_MITSHM=1

function returncode
{
  returncode=$?
  if [ $returncode != 0 ]; then
    echo "[$returncode]"
  else
    echo ""
  fi
}

if [[ ${EUID} == 0 ]]
then
	PS1='\[\033[0;31m\]$(returncode)\[\033[01;31m\]\u\[\033[00m\]@\[\033[01;33m\]\h\[\033[01;34m\] ${PWD} \$\[\033[00m\] '
else
	PS1='\[\033[0;31m\]$(returncode)\[\033[01;32m\]\u\[\033[00m\]@\[\033[01;33m\]\h\[\033[01;34m\] ${PWD} \$\[\033[00m\] '
fi

export BUILDDIR=/tmp
export MAKEFLAGS="-j$(nproc) $MAKEFLAGS"
export LD_LIBRARY_PATH="/usr/lib:$LD_LIBRARY_PATH"

#ROS Customization
source /opt/ros/noetic/setup.bash
source /root/noetic-dev/ros/interbotix_ws/devel/setup.bash
source /root/noetic-dev/ros/dependencies_ws/devel/setup.bash
source /root/noetic-dev/ros/homework_ws/devel/setup.bash


# # export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
# # if [ -z "$ROS_IP" ]; then
# #         export ROS_IP=127.0.0.1
# # fi
