from random import randrange
from parse import parse

def makeGridConnex(filepath):
    with open(filepath) as fp:
        line = fp.readline()
        line = fp.readline()
        edges = []
        while line:
            result = parse("({}, {})", line.strip())
            edges += [[result[0], result[1]], [result[1], result[0]]]
            line = fp.readline()

    fpw = open(filepath, "w")
    fpw.write(str(len(edges)) + '\n')
    for i in edges:
        fpw.write(("({}, {})\n").format(i[0], i[1]))
        
def createDensPopVelocity(filepath):
    with open(filepath) as fp:
        line = fp.readline()
        line = fp.readline()
        edges = []
        while line:
            result = parse("({}, {}, {})", line.strip())
            edges += [[result[0], result[1], result[2], randrange(100), randrange(120)]]
            line = fp.readline()
    fpw = open(filepath, "w")
    fpw.write(str(len(edges)) + '\n')
    for i in edges:
        fpw.write(("({}, {}, {}, {}, {})\n").format(i[0], i[1], i[2], i[3], i[4]))

createDensPopVelocity('./resources/graphs/16x16/nodes.txt')