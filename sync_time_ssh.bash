#!/bin/bash

printf "Now date is: %s\n" "$(date -u)"

if [ -z "$1" ]; then
  echo "Missing argument 1: username@hostname. Try: \"source $BASH_SOURCE ubuntu@192.168.50.135\""
else
  echo "Starting sync date with $1"
  sudo date --set="$(ssh $1 'date -u')" > /dev/null
fi

printf "Now date is: %s\n" "$(date -u)"

