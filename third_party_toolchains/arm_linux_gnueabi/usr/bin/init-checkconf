#!/bin/bash
#---------------------------------------------------------------------
# Script to determine if specified config file is valid or not.
# By default, two checks are performed:
#
#   - Ensure Upstart can parse overall file successfully
#   - Ensure all script sections are parseable by shell
#
#---------------------------------------------------------------------
#
# Copyright (C) 2011-2013 Canonical Ltd.
#
# Author: James Hunt <james.hunt@canonical.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3 of the License.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#---------------------------------------------------------------------

script_name=${0##*/}
confdir=$(mktemp -d /tmp/${script_name}.XXXXXXXXXX)
xdg_runtime_dir=$(mktemp -d /tmp/${script_name}.XXXXXXXXXX)
upstart_path=/sbin/init
initctl_path=/sbin/initctl
debug_enabled=n
file_valid=n
running=n
set_session=n
check_scripts=y

cleanup()
{
    # Restore
    [ -n "$saved_xdg_runtime_dir" ] && \
        debug "Restoring XDG_RUNTIME_DIR to '$saved_xdg_runtime_dir'"
    export XDG_RUNTIME_DIR="$saved_xdg_runtime_dir"

    [ -n "$saved_upstart_session" ] && \
        debug "Restoring UPSTART_SESSION to '$saved_upstart_session'"
    export UPSTART_SESSION="$saved_upstart_session"
  
    if [ ! -z "$upstart_pid" ]
    then
      debug "Stopping secondary Upstart (running with PID $upstart_pid)"
      kill -9 "$upstart_pid" >/dev/null 2>&1
    fi
  
    [ -d "$confdir" ] && rm -rf "$confdir"
    dir="$xdg_runtime_dir/upstart/sessions"
    [ -d "$dir" ] && rm -rf "$xdg_runtime_dir"
    [ "$file_valid" = y ] && exit 0
  
    exit 1
}

usage()
{
cat <<EOT
Description: Determine if specified Upstart (init(8)) job configuration
             file is valid.

Usage: $script_name [options] -f <conf_file>
       $script_name [options]    <conf_file>

Options:

 -d, --debug           : Show some debug output.
 -f <file>,            : Job configuration file to check.
 --file=<file>           (no default).
 -i <path>,            : Specify path to initctl binary
 --initctl-path=<path>   (default=$initctl_path).
 -s, --noscript        : Do not check script sections.
 -x <path>             : Specify path to init daemon binary
 --upstart-path=<path>   (default=$upstart_path).
 -h, --help            : Show this help.

EOT
}

debug()
{
    msg="$*"
    [ $debug_enabled = y ] && echo "DEBUG: $msg"
}

error()
{
    msg="$*"
    printf "ERROR: %s\n" "$msg" >&2
}

die()
{
    error "$*"
    exit 1
}

# Return 0 if Upstart is running, else 1
upstart_running()
{
    initctl --user version >/dev/null 2>&1
}

trap cleanup EXIT INT TERM

args=$(getopt \
    -n "$script_name" \
    -a \
    --options="df:hi:sx:" \
    --longoptions="debug file: help initctl-path: noscript upstart-path:" \
    -- "$@")

eval set -- "$args"
[ $? -ne 0 ] && { usage; exit 1; }
[ $# -eq 0 ] && { usage; exit 0; }

while [ $# -gt 0 ]
do
    case "$1" in
      -d|--debug)
        debug_enabled=y
        ;;

      -f|--file)
        file="$2"
        shift
        ;;

      -h|--help)
        usage
        exit 0
        ;;

      -i|--initctl-path)
        initctl_path="$2"
        shift
        ;;

      -s|--noscript)
        check_scripts=n
        ;;

      -x|--upstart-path)
        upstart_path="$2"
        shift
        ;;

      --)
        shift
        break
        ;;
    esac
    shift
done

[ -z "$file" ] && file="$1"
[ -z "$file" ] && die "Must specify configuration file"
[ ! -f "$file" ] && die "File $file does not exist"

debug "upstart_path=$upstart_path"
debug "initctl_path=$initctl_path"

for cmd in "$upstart_path" "$initctl_path"
do
    [ -f "$cmd" ] || die "Path $cmd does not exist"
    [ -x "$cmd" ] || die "File $cmd not executable"
    "$cmd" --help | grep -q -- --user || die "version of $cmd too old"
done

export saved_xdg_runtime_dir="$XDG_RUNTIME_DIR"
debug "Setting XDG_RUNTIME_DIR='$xdg_runtime_dir'"
export XDG_RUNTIME_DIR="$xdg_runtime_dir"

export saved_upstart_session="$UPSTART_SESSION"
[ -n "$UPSTART_SESSION" ] \
    && debug "Unsetting UPSTART_SESSION ($UPSTART_SESSION)" \
    && unset UPSTART_SESSION

# this is the only safe way to run another instance of Upstart
"$upstart_path" --help|grep -q -- --no-startup-event || die "$upstart_path too old"

debug "confdir=$confdir"
debug "file=$file"

filename=$(basename $file)

echo "$filename" | egrep -q '\.conf$' || die "File must end in .conf"

job="${filename%.conf}"

cp "$file" "$confdir" || die "Failed to copy file $file to $confdir"
debug "job=$job"

upstart_out="$(mktemp --tmpdir "${script_name}-upstart-output.XXXXXXXXXX")"
debug "upstart_out=$upstart_out"

upstart_cmd=$(printf \
    "%s --user --no-dbus --no-startup-event --verbose --confdir %s" \
        "$upstart_path" \
        "$confdir")
debug "upstart_cmd=$upstart_cmd"

nohup $upstart_cmd >"$upstart_out" 2>&1 &
upstart_pid=$!
debug "Upstart pid=$upstart_pid"

# Stop the shell outputting a message when Upstart is killed.
# We handle this ourselves in cleanup().
disown 

# wait for Upstart to initialise
for i in $(seq 1 5)
do
    sessions=$("$initctl_path" list-sessions)

    if [ "$set_session" = n ] && [ -n "$sessions" ]
    then
        count=$(echo "$sessions"|wc -l)
        [ "$count" -gt 1 ] && die "Got unexpected session count: $count"
        session=$(echo "$sessions"|awk '{print $2}')
        debug "Joining Upstart session '$session'"
        export UPSTART_SESSION="$session"
	set_session=y
    fi

    debug "Waiting for Upstart to initialise (attempt $i)"

    upstart_running
    if [ $? -eq 0 ]
    then
        running=y
        break
    fi

    sleep 1
done

[ $running = n ] && die "Failed to ask Upstart to check conf file"

debug "Secondary Upstart ($upstart_cmd) running with PID $upstart_pid"

if [ "$check_scripts" = y ]
then
    for section in pre-start post-start script pre-stop post-stop
    do
        if egrep -q "\<${section}\>" "$file"
        then
            cmd='sed -n "/^ *${section}/,/^ *end script/p" $file | /bin/sh -n 2>&1'
            errors=$(eval "$cmd")
            [ $? -ne 0 ] && \
                die "$(printf "File $file: shell syntax invalid in $section section:\n${errors}")"
        fi
    done
fi

"$initctl_path" --user list|grep -q "^${job}"
if [ $? -eq 0 ]
then
    file_valid=y
    echo "File $file: syntax ok"
    exit 0
fi

errors=$(grep "$job" "$upstart_out"|sed "s,${confdir}/,,g")
die "$(printf "File $file: syntax invalid:\n${errors}")"
