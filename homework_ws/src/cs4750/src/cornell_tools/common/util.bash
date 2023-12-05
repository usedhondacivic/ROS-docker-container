# Networking

###
# Pulls the IP address for a given network interface
##
function get_ip(){
  if [[ $# != 1 ]]; then
      echo "Usage: $0 network_if"
      return 1
  fi
  network_if="$1"

  target_ip=$(LANG=C ip addr show | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*.*en.*' | grep -Eo '([0-9]*\.){3}[0-9]*' | head -1 ) &> /dev/null
  # We may get degenerate strings back. Filter them out
  if [[ "${#target_ip}" -le 5 ]] ; then
      return 1
  else
      echo "$target_ip"
      return 0
  fi
}

function set_rosip {
    ip_address="$(get_ip "$1")"
    return_code=$?
    if [[ "$return_code" != "0" ]] ; then
        # NOTE(nickswalker4-14-21): Disabled until we need to configure graph for robot use
        #>&2 echo "[CORNELL Tools] Could not get IP address from network interface eno1. Will not set ROS_IP. Use set_network_if to configure which network interface to use with ROS"
        return 1
    else
        export ROS_IP="$ip_address"
    fi
}

ROS_PROMPT_MASTER_UP="up"
ROS_PROMPT_MASTER_DOWN="dwn"
ROS_PROMPT_MASTER_OUT="timeout"
ROS_PROMPT_MASTER_UNEXPECTED="err"
ROS_PROMPT_USE_SIM_TIME="sim_time"

function get_ros_status() {
    ros_status=""
    LC_ALL=C curl -m 0.1 -s -o /dev/null \
    --request POST "$ROS_MASTER_URI/RPC2" --data "<?xml version='1.0'?>
    <methodCall><methodName>getUri</methodName><params><param><value><string>
    </string></value></param></params></methodCall>"
    case "$?" in
        0)
            # master up and responding
            [ "$(rosparam get use_sim_time 2>&1 | grep 'true')" ] && \
            ros_status+="$ROS_PROMPT_USE_SIM_TIME "
            ros_status+="$ROS_PROMPT_MASTER_UP"
        ;;
        7)
            # master down and machine responding
            ros_status+="$ROS_PROMPT_MASTER_DOWN"
        ;;
        6|28)
            # Could not resolve host
            # Connection timeout
            ros_status+="$ROS_PROMPT_MASTER_OUT"
        ;;
        *)
            # Unexpected
            ros_status+="$ROS_PROMPT_MASTER_UNEXPECTED[$?]"
        ;;
    esac
    echo "$ros_status"
}