import sys
comm=[xx.strip() for xx in open(sys.argv[1], 'r')]
sil_symbol=sys.argv[2]
fout=open(sys.argv[3], 'w')

fout.writelines("0 1 %s %s 0\n" % (sil_symbol, sil_symbol))
fout.writelines("2 3 %s %s 0\n" % (sil_symbol, sil_symbol))
fout.writelines("3 0\n")

state=4
for line in comm:
    fout.writelines("1 %s <eps> <eps> 0\n" % (str(state)))
    words=line.split()
    for word in words:
	fout.writelines("%s %s %s %s 0\n" % (str(state), str(state+1), word, word))
	state+=1
    fout.writelines("%s 2 <eps> <eps> 0\n" % (str(state)))
    state+=1
fout.close()
    
