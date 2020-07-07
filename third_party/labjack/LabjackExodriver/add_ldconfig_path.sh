#! /bin/bash

ldconfig_file=/etc/ld.so.conf
path=/usr/local/lib
TRUE=0
path_exists=1

for line in `cat $ldconfig_file`; do
	if [ $line == $path ]; then
		path_exists=$TRUE
	fi
done

if [ $path_exists -ne $TRUE ]; then
	echo "$path  >> $ldconfig_file"
	echo $path  >> $ldconfig_file
fi

