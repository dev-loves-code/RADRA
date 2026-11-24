import re,csv,collections
def blocks(path):
    out=[]
    for line in open(path):
        m=re.search(r'<block position="(\d+),(\d+),(\d+)"',line)
        if not m: continue
        r=re.search(r'region="(\d+)"',line); f='fail="true"' in line
        out.append(((int(m[1]),int(m[3])),int(r[1]),f))
    return out  # id = index+1
def events(path):
    return list(csv.DictReader(open(path)))
