#!/usr/bin/bash

cp $1 $1.tcp

s=$(echo $(crc32 $1)$(echo "!"))
sed -i "1s/^/$s/" $1.tcp

netcat -N 192.168.86.102 50005 <$1.tcp
