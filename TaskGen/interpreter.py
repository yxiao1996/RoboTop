import yaml

def deleteEmptyLines(lines):
    out_lines = []
    for i in range(len(lines)):
        if len(lines[i]) != 1:
            line = lines[i].split('\n')[0]
            out_lines.append(line)
    return out_lines

def deleteComments(lines):
    out_lines = []
    for i in range(len(lines)):
        line = lines[i].split('#')[0]
        if len(line) > 0:
            # delete spaces in the end
            while(line[-1] == ' '):
                line = line[:-1]
            out_lines.append(line)
    return out_lines

def matchMacro(symbols):
    assert len(symbols) == 5
    if symbols[0] == 'm':
        if symbols[1] == 'a':
            if symbols[2] == 'c':
                if symbols[3] == 'r':
                    if symbols[4] == 'o':
                        return True
                    else:
                        return False
                else:
                    return False
            else:
                return False
        else:
            return False
    else:
        return False

def findMacro(lines):
    macro_pos = []
    for i in range(len(lines)): 
        # for each line, split and match
        line = lines[i]
        #line = line.split()
        for j in range(len(line)-4):
            symbols = []
            for k in range(5):
                symbols.append(line[j+k])
            if matchMacro(symbols):
                macro_pos.append(i)
    return  macro_pos

def taskParser(lines, macro_pos):
    segments = []
    for i in range(len(macro_pos)-1):
        seg_start = macro_pos[i] + 1
        seg_end = macro_pos[i+1] - 1
        segment = []
        for j in range(seg_end-seg_start+1):
            segment.append(lines[seg_start+j])
        segments.append(segment)
    return segments

def Tokenizer(lines):
    lines = deleteComments(deleteEmptyLines(lines))
    print lines
    macro_pos = findMacro(lines)
    segments = taskParser(lines, macro_pos)
    macro_tasks = {}
    for i in range(len(segments)):
        print i
        macro_task = []
        segment = segments[i]
        # tokenize each segemnt
        for j in range(len(segment)):
            # tokenize each line
            line = segment[j]
            # Parse the code with bracket, get pose
            pose = []
            tmp = line.split('[')[1]
            tmp = tmp.split(']')[0]
            tmp = tmp.split(',')
            for k in range(len(tmp)):
                pose.append(float(tmp[k]))
            # Parse the leftover get move
            tmp = line.split(']')[1]
            move = tmp.split(' ')[-1]
            #print pose, move
            task = {'move': move,
                    'path': [pose]}
            macro_task.append(task)
        macro_name = 'macro_task_'+str(i)
        macro_tasks[macro_name] = macro_task
    return macro_tasks

def Interpreter():
    save_dir = '../catkin_ws/src/robocon/tasks/'
    f = open('./sample.txt')
    lines = []
    lines = f.readlines()
    filename = lines[0].split('\n')[0] + '.yaml'
    macro_tasks = Tokenizer(lines)
    data = {"macro_tasks": macro_tasks}
    # Dump yaml file
    stream = file(save_dir + filename, 'w')
    yaml.dump(data, stream)    # Write a YAML representation of data to 'document.yaml'.
    print yaml.dump(data)      # Output the document to the screen.

Interpreter()