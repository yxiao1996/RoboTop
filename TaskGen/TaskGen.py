import yaml

save_dir = '../catkin_ws/src/robocon/tasks/'
# Open txt file
f = open('./sample.txt')
# read fist line as file name
filename = f.readline().split('\n')[0] + '.yaml'
# Convert txt to tasks
macrotasks = []
macrotask_index = 0
init = False
while True:
    try:
        # Try to read the next line
        line = f.next()
        #print line
        # Remove the last byte
        line = line.split('\n')[0]
        # check new macro task
        if line == "macro":
            # first macro task?
            if not init:
                current_macro = []
                init = True
            else:
                # append macro task
                #print current_macro
                macro_name = 'macro_task_'+str(macrotask_index)
                data = {macro_name: current_macro}
                macrotasks.append(data)
                macrotask_index += 1
                current_macro = []
            line = f.next()
            line = line.split('\n')[0]
        # Parse the code with bracket, get pose
        pose = []
        tmp = line.split('[')[1]
        tmp = tmp.split(']')[0]
        tmp = tmp.split(',')
        for i in range(len(tmp)):
            pose.append(float(tmp[i]))
        # Parse the leftover get move
        tmp = line.split(']')[1]
        move = tmp.split(' ')[-1]
        #print pose, move
        task = {'path': pose,
                'move': move}
        #print task
        current_macro.append(task)
    except:
        break

data = {"macro_tasks": macrotasks}
# Dump yaml file
print save_dir+filename
stream = file(save_dir + filename, 'w')
yaml.dump(data, stream)    # Write a YAML representation of data to 'document.yaml'.
print yaml.dump(data)      # Output the document to the screen.