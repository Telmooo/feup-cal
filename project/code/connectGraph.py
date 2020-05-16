from parse import parse

def graph(filepath):
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

graph('edges.txt')