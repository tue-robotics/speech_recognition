#!/bin/bash

ip=$1
op=$2


rm tmp*
awk '{print $1}' $ip > tmp
no=`head -1 tmp | grep -o "/" | wc -l`
nv=$((no+1)); 
cut -f $nv -d "/" tmp | sed -e "s|.wav||g" > tmp1
paste tmp1 tmp > $op

