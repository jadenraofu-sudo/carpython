import matplotlib.pyplot as plt
import time
import sys
from config import environment
if environment == 'mac':
    from plot import line, semi
elif environment == 'car':
    from moves import line, semi
    import Rpi.GPIO
else:
    raise Exception("Invalid environment")


available_functions = ['line', 'semi']
immediate_functions = ['relativeTime', 'startQ', 'setpoints']
scheduled_actions = []
points = []
starting = 0.0
relative = True  # true for incremental time (default), False for relative time
started = False
nextStart = 0.0


def parse(actionList):
    global nextStart

    startAt = 0

    for i in actionList:
        print(i, end='')
        startAt = nextStart
        if not i.startswith('#'):
            j = i.split(' ', 1)
            func = j[1].split('(', 1)
            if func[0] in immediate_functions:
                print("executing ", j[1])
                exec(j[1])
            else:
                if relative:
                    nextStart += float(j[0])
                else:
                    nextStart = float(j[0])
                scheduled_actions.append((startAt, j[1]))


def drive():
    print("hi")


# immediate functions
def setpoints(pts):
    global points
    points = pts
    print(f"@{time.time()-starting:.2f} points set to {points}")


def relativeTime(arg):
    global relative
    relative = arg
    if relative:
        print(f"@{time.time()-starting:.2f} Interpretated as relative times")
    else:
        print(f"@{time.time()-starting:.2f}",
              "Interpretated as absolute times, starting now")


def startQ(arg):
    global started
    if started:
        return
    started = True
    print(f"@{time.time()-starting:.2f} Executing q", arg)

    # sort actions by scheduled time
    scheduled_actions.sort(key=lambda x: x[0])

    if environment == 'mac':
        for startAt, cmd in scheduled_actions:
            while time.time() - starting < startAt:
                plt.pause(0.01)   # GUI-safe sleep
            print(f"@{time.time()-starting:.2f} starting:", cmd)
            run_a_func_from_string(cmd)
    plt.show()


def run_a_func_from_string(arg):  # helper function to check if registered
    func = arg.split('(', 1)
    if func[0] in available_functions:
        exec(arg)


def main():
    global starting
    print("starting main, using file list of functions")

    if len(sys.argv) == 1:
        myfile = 'instructions.txt'
    else:
        myfile = sys.argv[1]
    print("reading file ", myfile)

    with open(myfile, encoding="utf-8") as myf:
        actionList = myf.readlines()

    starting = time.time()

    parse(actionList)
    print("input file commands added")
    time.sleep(5)  # to see if q started by immediate command
    if not started:
        startQ("")  # argument needed
    # stop_car()  # stop movement
    # destroy()   # clean up GPIO
    print(f"@{time.time()-starting:.2f} Completed and cleanup done")


if __name__ == '__main__':
    starting = time.time()
    try:
        main()
    except KeyboardInterrupt:
        # stop_car() # stop movement
        # destroy()  # clean up GPIO
        print(f"@{time.time()-starting:.2f} Stopped by user, cleanup done")
