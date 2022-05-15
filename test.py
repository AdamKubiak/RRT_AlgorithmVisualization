import re
with open("twin.txt",encoding="utf-8") as file_in:
    lines = []
    for line in file_in:
        lines.append(line)
i = 6
newdoclist= []
print(lines[i])
print(lines[i][6])
for line in lines:
    if re.search('00:',line):
        dziesiatki = int(float(line[6]))*10
        jednosci = int(float(line[7]))
        dziesiatki1 = int(float(line[23]))*10
        jednosci1 = int(float(line[24]))
        wynik = str(dziesiatki+jednosci+5)
        wynik1 = str(dziesiatki1+jednosci1+5)
        if int(wynik) < 10:
            wynik = '0'+wynik
        
        if int(wynik1) < 10:
            wynik1 = '0'+wynik1
        if int(wynik) > 60:
            wynik = '60'

        if int(wynik1) > 60:
            wynik1 = '60'
        przod = line[:6]
        srodek = line[8:23]
        tyl = line[25:]
        NEWLINE = przod+wynik+srodek+wynik1+tyl
        newdoclist.append(NEWLINE)
    else: 
        newdoclist.append(line)


with open('your_file.txt', 'w',encoding="utf-8") as f:
    for item in newdoclist:
        f.write("%s" % item)

#print(line)