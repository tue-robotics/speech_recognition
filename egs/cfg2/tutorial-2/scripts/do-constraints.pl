#!/usr/bin/perl

$file=$ARGV[0];
foreach $letter ("A".."Z") {
    $doitstr.="fstintersect fwdconstr/$letter.fsa $file | fstdeterminize | fstminimize ";
    $doitstr.="| fstintersect revconstr/$letter.fsa $file | fstdeterminize | fstminimize ";
    if ($letter ne "Z") {
	$doitstr.="|";
    }
    $file="-"
}

system($doitstr);
