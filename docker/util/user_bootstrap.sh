#!/bin/bash

set -e

uid="$(ls -nd . | awk '{print $3}')"
gid="$(ls -nd . | awk '{print $4}')"

user="docker"
group="docker"

# Create an account and run as that account.
groupadd "$group" --non-unique -g "$gid"
useradd "$user" --non-unique -m -u "$uid" -g "$gid"
chown $user:$group /home/$user
echo "ALL ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
install -dm755 -o "$uid" -g "$gid" /run/user/"$uid"

if [ -d /venv ]; then
    source /venv/bin/activate
fi

export HOME="/home/$user"
exec runuser -g "$group" -u "$user" -- /bin/bash -c "$@"
