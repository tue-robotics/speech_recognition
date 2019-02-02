#!bin/sh

# Based on http://www.isle.illinois.edu/sst/courses/minicourses/2009/lecture6.pdf

bash compileAndDraw.sh sent_one.fsa
bash compileAndDraw.sh dict.fst

fstcompose --fst_compat_symbols=false sent_one.fsa dict.fst > strings_one.fst
fstdraw --portrait  strings_one.fst | dot -Tsvg >  strings_one.svg
echo 'Done composing: outputted strings.svg'
echo 'Example sentences:'
echo '------------------'

for i in `seq 1 10`;
do
	fstrandgen --seed=$RANDOM strings_one.fst | fstproject --project_output |
	fstprint --acceptor --isymbols=dict.syms |
	awk '{printf("%s ",$3)}END{printf("\n")}'
done
