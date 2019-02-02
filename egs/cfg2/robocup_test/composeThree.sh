#!bin/sh

# Based on http://www.isle.illinois.edu/sst/courses/minicourses/2009/lecture6.pdf

bash compileAndDraw.sh sent_three.fsa
bash compileAndDraw.sh dict.fst

fstcompose --fst_compat_symbols=false sent_three.fsa dict.fst > strings_three.fst
fstdraw --portrait  strings_three.fst | dot -Tsvg >  strings_three.svg
echo 'Done composing: outputted strings.svg'
echo 'Example sentences:'
echo '------------------'

for i in `seq 1 10`;
do
	fstrandgen --seed=$RANDOM strings_three.fst | fstproject --project_output |
	fstprint --acceptor --isymbols=dict.syms |
	awk '{printf("%s ",$3)}END{printf("\n")}'
done
