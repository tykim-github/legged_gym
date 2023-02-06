# f = open('./urdf/ver4_0.txt', 'r')
f = open('./hopper.urdf', 'r')
content = f.read()
# print(content)
f.close()

newlines = ''
segment = 0
i_before = ''
for i in content:
    if i == '<':
        segment = 1
        # print(i)
    elif i == '>':
        segment = 0
    if segment == 1:
        # print('hi')
        if i == '\n':
            pass
        elif i == ' ' and i_before == ' ':
            pass
        else:
            newlines += i
    else:
        newlines += i
    i_before = i

line_arr = newlines.split('\n')

joint_start = []
joint_end = []
check = 0

newnew = ''
newnew_joint = ''
for idx, i in enumerate(line_arr):
    if '<joint name' in i and "type=\"revolute\"" in i:
        newnew_joint += i + '\n'
        check = 1
        joint_start.append(idx)
    elif check == 1:
        newnew_joint += i + '\n'
        if "</joint>" in i:
            check = 0
            joint_end.append(idx)
    elif '</robot>' in i:
        continue
    else:
        newnew += i + '\n'

wholenew = newnew + newnew_joint + '</robot>'

line_arr = wholenew.split('\n')

links_to_preserve_collision = ["pelvis","foot"]
newnew = ''
newnew_joint = ''
nnn = ''
preserve = 0
collision = 0
keep = True
for idx, i in enumerate(line_arr):
    for ii in links_to_preserve_collision:
        if ii in i and '<link name' in i:
            preserve = 1
    if '<collision>' in i:
            collision = 1
    if collision == 1:
        if preserve == 1:
            keep = True
        else:
            keep = False
    else:
        keep = True        
    if collision == 1:
        if '</collision>' in i:
            collision = 0
    if preserve == 1 and '</link>' in i:
        preserve = 0
    
    if keep:
        nnn += i + '\n'
print(nnn)