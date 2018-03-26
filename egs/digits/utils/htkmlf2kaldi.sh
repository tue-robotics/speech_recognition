#!/bin/bash

ip=$1
op=$2

sed -i '$ d' $ip
sed ':a;N;$!ba;s/\n/ /g' $ip | sed -e 's|\. "*|\n"*|g;s|\"\*\*.||g;s|.lab\"||g;s|#!MLF!#||g;s|\#\!MLF\!\#||g;s| \"\*/||g' > $op
